#include <Arduino.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include <FT6336.h>
#include <WiFi.h>
#include <Preferences.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_sleep.h>
#include <driver/gpio.h>
#include <driver/rtc_io.h>
#include <esp32-hal-rgb-led.h>
#include <atomic>
#include "remote_app.h"
#include "WallEProtocol.h"
#include "ui_assets.h"
#include "build_info.h"

namespace {
using walle::Packet;using walle::Kind;using walle::Direction;
struct Rect{int16_t x,y,w,h;bool contains(int px,int py)const{return px>=x&&py>=y&&px<x+w&&py<y+h;}};
constexpr Rect BUTTONS_SINGLE[]={{108,76,100,80},{108,232,100,80},{8,154,100,80},{208,154,100,80}};
constexpr Rect BUTTONS_DUAL[]={{16,88,112,96},{16,208,112,96},{232,208,112,96},{352,208,112,96}};
constexpr Rect LINK={14,14,190,36},CLOSE={380,50,40,40};
constexpr Rect BATTERY={372,14,94,36},FLIP={74,100,332,48},MODE={74,164,332,48};
constexpr uint8_t BROADCAST[]={255,255,255,255,255,255};
constexpr uint8_t LAST_SCAN_CHANNEL=11;
constexpr uint32_t LONG_PRESS_MS=1800,WAKE_HOLD_MS=1500;
struct RxEvent{uint8_t mac[6];Packet packet;int8_t rssi;uint8_t channel;};
struct Candidate{bool used=false;uint8_t mac[6]{};char name[17]{};uint8_t channel=1;int8_t rssi=0;uint32_t seen=0;};
bool sameMac(const uint8_t *a,const uint8_t *b){return memcmp(a,b,6)==0;}

class RemoteApp {
 public:
 static RemoteApp *instance;
 TFT_eSPI lcd;
 FT6336 touchSensor{16,15,17,18,320,480};
 Preferences prefs;
 QueueHandle_t rxQueue=nullptr;
 std::atomic_bool txBusy{false};
 bool displayReady=false,radioReady=false,protocolOK=false;
 bool linked=false,pairPending=false,haveStored=false,manualPairing=false;
 uint8_t storedMac[6]{},peerMac[6]{};
 uint8_t storedChannel=6,currentChannel=1,peerChannel=1;
 Candidate candidates[3];
 uint32_t nonce=0,session=0,sequence=0;
 uint32_t scanStarted=0,lastDiscover=0,lastPairSend=0,pairStarted=0,lastAlive=0,lastControl=0,txStarted=0;
 uint32_t introEnd=0,lastTouchPoll=0,lastTouchGood=0,lastBattery=0,lastLog=0,bootPressedAt=0;
 bool intro=true,modal=false,uiDirty=false,touchWasDown=false,inputArmed=false,forceControl=false;
 bool settings=false,screenFlipped=false,backlightPending=false;
 bool twoHand=true;
 const Rect *buttons()const{return twoHand?BUTTONS_DUAL:BUTTONS_SINGLE;}
 bool bootGuard=true,bootDown=false;
 Direction desired=Direction::Stop;
 int visualButton=-1,paintedButton=-1,paintedStatus=-1,batteryPercent=-1,paintedBattery=-100;
 uint16_t batteryMv=0;
 int previousTouchCount=-1;
 uint32_t lastTouchDiagnostic=0;
 uint32_t introStarted=0,lastIntroFrame=0,introFrames=0,introMaxDrawUs=0;
 int introColumns=0;
 uint16_t introStrip[16*320];

 static void received(const esp_now_recv_info_t *info,const uint8_t *data,int len){
  if(!instance||!instance->rxQueue||!info||!info->src_addr||len!=(int)sizeof(Packet))return;
  RxEvent event{};memcpy(event.mac,info->src_addr,6);memcpy(&event.packet,data,sizeof(Packet));
  if(info->rx_ctrl){event.rssi=info->rx_ctrl->rssi;event.channel=info->rx_ctrl->channel;}
  xQueueSend(instance->rxQueue,&event,0);
 }
 static void sent(const esp_now_send_info_t *,esp_now_send_status_t){if(instance)instance->txBusy.store(false);}

 bool setChannel(uint8_t channel){
  if(channel<1||channel>LAST_SCAN_CHANNEL)return false;
  if(esp_wifi_set_channel(channel,WIFI_SECOND_CHAN_NONE)!=ESP_OK)return false;
  currentChannel=channel;return true;
 }
 bool addPeer(const uint8_t *mac){
  if(esp_now_is_peer_exist(mac))return true;
  esp_now_peer_info_t peer{};memcpy(peer.peer_addr,mac,6);peer.channel=0;peer.ifidx=WIFI_IF_STA;peer.encrypt=false;
  return esp_now_add_peer(&peer)==ESP_OK;
 }
 bool sendPacket(Kind kind,const uint8_t *mac,int16_t throttle=0,int16_t steering=0){
  if(!radioReady||txBusy.load())return false;
  Packet p{};p.kind=kind;p.sequence=++sequence;p.session=kind==Kind::Discover?0:session;p.nonce=nonce;
  p.throttle=throttle;p.steering=steering;p.batteryMv=batteryMv;p.leaseMs=walle::CONTROL_LEASE_MS;
  memcpy(p.name,"WALLE_REMOTE",12);walle::seal(p);
  txBusy.store(true);txStarted=millis();
  esp_err_t result=esp_now_send(mac,(const uint8_t*)&p,sizeof(p));
  if(result!=ESP_OK){txBusy.store(false);return false;}return true;
 }
 void requestStop(){desired=Direction::Stop;visualButton=-1;forceControl=true;inputArmed=false;}
 void beginScan(bool manual){
  requestStop();linked=false;pairPending=false;manualPairing=manual;
  for(auto &c:candidates)c.used=false;
  if(radioReady)setChannel(1);scanStarted=millis();lastDiscover=0;uiDirty=true;
  Serial.printf("DISCOVERY_START channels=1-%u manual=%u\r\n",LAST_SCAN_CHANNEL,manual);
 }
 void beginPair(const uint8_t *mac,uint8_t channel){
  requestStop();if(!radioReady||!setChannel(channel)||!addPeer(mac))return;
  if(!sameMac(peerMac,mac)&&esp_now_is_peer_exist(peerMac))esp_now_del_peer(peerMac);
  memcpy(peerMac,mac,6);peerChannel=channel;session=esp_random();if(!session)session=1;
  linked=false;pairPending=true;pairStarted=millis();lastPairSend=0;uiDirty=true;
  Serial.printf("PAIR_REQUEST peer=%02X:%02X:%02X:%02X:%02X:%02X channel=%u\r\n",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5],channel);
 }
 void onOffer(const RxEvent &e,const Packet &p){
  if(p.nonce!=nonce||e.channel<1||e.channel>LAST_SCAN_CHANNEL||pairPending||linked)return;
  int slot=-1;for(int i=0;i<3;i++)if(candidates[i].used&&sameMac(candidates[i].mac,e.mac)){slot=i;break;}
  if(slot<0)for(int i=0;i<3;i++)if(!candidates[i].used){slot=i;break;}
  if(slot>=0){
   Candidate &c=candidates[slot];bool changed=!c.used||!sameMac(c.mac,e.mac);
   c.used=true;memcpy(c.mac,e.mac,6);memcpy(c.name,p.name,16);c.name[16]=0;
   for(char &ch:c.name)if(ch&&(ch<32||ch>126))ch='?';
   if(!c.name[0])strcpy(c.name,"WALL-E");c.channel=e.channel;c.rssi=e.rssi;c.seen=millis();
   if(changed){uiDirty=true;Serial.printf("ROBOT_FOUND name=%s channel=%u rssi=%d\r\n",c.name,c.channel,c.rssi);}
  }
  if(haveStored&&!manualPairing&&sameMac(storedMac,e.mac))beginPair(e.mac,e.channel);
 }
 void radioTick(){
  if(!radioReady)return;uint32_t now=millis();
  if(txBusy.load()&&now-txStarted>150)txBusy.store(false);
  RxEvent e{};
  while(xQueueReceive(rxQueue,&e,0)==pdTRUE){
   Packet p{};if(!walle::decode(&e.packet,sizeof(e.packet),p))continue;
   if(p.kind==Kind::Offer){onOffer(e,p);continue;}
   if((pairPending||linked)&&sameMac(e.mac,peerMac)&&p.session==session&&p.nonce==nonce){
    if(p.kind==Kind::PairAck&&pairPending){
     pairPending=false;linked=true;lastAlive=now;lastControl=0;requestStop();
     memcpy(storedMac,peerMac,6);storedChannel=peerChannel;haveStored=true;
     prefs.putBytes("peer",storedMac,6);prefs.putUChar("channel",storedChannel);
     modal=false;manualPairing=false;uiDirty=true;
     Serial.println("PAIR_CONFIRMED application_ack=1");
    }else if(p.kind==Kind::Heartbeat&&linked){lastAlive=now;}
   }
  }
  for(auto &c:candidates)if(c.used&&now-c.seen>5500){c.used=false;if(modal)uiDirty=true;}
  if(linked){
   if(now-lastAlive>walle::REMOTE_LINK_TIMEOUT_MS){Serial.println("LINK_TIMEOUT stop=1");beginScan(false);return;}
   if(forceControl||now-lastControl>=walle::CONTROL_PERIOD_MS){
    int16_t throttle=0,steering=0;walle::axes(desired,throttle,steering);
    if(sendPacket(Kind::Control,peerMac,throttle,steering)){lastControl=now;forceControl=false;}
   }
   return;
  }
  if(pairPending){
   if(now-pairStarted>1500){Serial.println("PAIR_TIMEOUT");beginScan(modal);return;}
   if(!lastPairSend||now-lastPairSend>=250)if(sendPacket(Kind::PairRequest,peerMac))lastPairSend=now;
   return;
  }
  if(!txBusy.load()&&now-scanStarted>=200){
   setChannel(currentChannel>=LAST_SCAN_CHANNEL?1:currentChannel+1);scanStarted=now;lastDiscover=0;
  }
  if(now-scanStarted>=20&&(!lastDiscover||now-lastDiscover>=75))if(sendPacket(Kind::Discover,BROADCAST))lastDiscover=now;
 }
 void initRadio(){
  if(!protocolOK)return;
  rxQueue=xQueueCreate(8,sizeof(RxEvent));if(!rxQueue)return;
  WiFi.mode(WIFI_STA);WiFi.setAutoReconnect(false);WiFi.disconnect(false,false);WiFi.setSleep(false);
  nonce=esp_random();if(!nonce)nonce=1;
  if(esp_now_init()!=ESP_OK)return;
  esp_now_register_recv_cb(received);esp_now_register_send_cb(sent);
  radioReady=true;if(!addPeer(BROADCAST)){radioReady=false;return;}
  Serial.printf("ESPNOW_READY mac=%s protocol=%u\r\n",WiFi.macAddress().c_str(),walle::VERSION);
  if(haveStored)beginPair(storedMac,storedChannel);else beginScan(false);
 }
 int statusIndex()const{return !radioReady?4:linked?2:pairPending?1:haveStored?3:0;}
 void push(int x,int y,int w,int h,const uint16_t *pixels){lcd.setSwapBytes(true);lcd.pushImage(x,y,w,h,pixels);}
 void paintButton(int id,bool pressed=true){
  if(id<0||id>3)return;
  const uint16_t *active[]={button_pressed_0,button_pressed_1,button_pressed_2,button_pressed_3,button_pressed_4,button_pressed_5,button_pressed_6,button_pressed_7};
  const uint16_t *normal[]={button_normal_0,button_normal_1,button_normal_2,button_normal_3,button_normal_4,button_normal_5,button_normal_6,button_normal_7};
  const Rect &r=buttons()[id];int asset=(twoHand?4:0)+id;push(r.x,r.y,r.w,r.h,pressed?active[asset]:normal[asset]);
 }
 void paintStatus(){
  const uint16_t *images[]={link_state_0,link_state_1,link_state_2,link_state_3,link_state_4};
  paintedStatus=statusIndex();push(14,14,190,36,images[paintedStatus]);
 }
 void paintBattery(){
  push(372,14,94,36,battery_glass);uint16_t color=batteryPercent>=0&&batteryPercent<15?0xFD68:0xF7DF;
  lcd.drawRoundRect(384,25,23,13,3,color);lcd.fillRect(407,29,2,5,color);
  if(batteryPercent>0)lcd.fillRect(387,28,max(1,batteryPercent*17/100),7,color);
  char value[8];if(batteryPercent<0)strcpy(value,"--");else snprintf(value,sizeof(value),"%d%%",batteryPercent);
  lcd.setTextFont(2);lcd.setTextSize(1);lcd.setTextColor(color);lcd.setTextDatum(ML_DATUM);lcd.drawString(value,415,31);
  paintedBattery=batteryPercent;
 }
 int candidateCount()const{int count=0;for(const auto &c:candidates)if(c.used)count++;return count;}
 int candidateForRow(int row)const{for(int i=0;i<3;i++)if(candidates[i].used){if(row--==0)return i;}return -1;}
 void paintModal(){
  if(!candidateCount()){push(0,0,480,320,pair_empty);return;}
  push(0,0,480,320,pair_list);int row=0;
  for(const auto &c:candidates)if(c.used){
   int y=100+row*48;lcd.setTextColor(0xF7DF);lcd.setTextDatum(TL_DATUM);lcd.setTextFont(2);lcd.setTextSize(1);
   lcd.drawString(c.name,84,y+3);char mac[24];snprintf(mac,sizeof(mac),"%02X:%02X:%02X:%02X:%02X:%02X",c.mac[0],c.mac[1],c.mac[2],c.mac[3],c.mac[4],c.mac[5]);
   lcd.setTextFont(1);lcd.setTextColor(0xAE3A);lcd.drawString(mac,84,y+27);row++;
  }
 }
 void paintSettings(){
  push(0,0,480,320,settings_page);
  push(74,164,332,48,twoHand?mode_row_1:mode_row_0);
  lcd.setTextFont(2);lcd.setTextSize(1);lcd.setTextDatum(MC_DATUM);lcd.setTextColor(0xF7DF);
  lcd.drawString(WALLE_BUILD_REV " by Azucia",240,249);
  lcd.setTextColor(0xAE3A);lcd.drawString(__DATE__ " " __TIME__,240,271);
 }
 void animateIntro(){
  uint32_t now=millis();
  if(now-lastIntroFrame<16||introColumns>=480)return;
  lastIntroFrame=now;
  int target=min(480,(int)((uint64_t)(now-introStarted)*480/1200));
  if(target<=introColumns)return;
  uint32_t drawStart=micros();
  while(introColumns<target){
   int width=min(16,target-introColumns);
   for(int row=0;row<320;row++)memcpy(introStrip+row*width,boot_logo+row*480+introColumns,width*sizeof(uint16_t));
   push(introColumns,0,width,320,introStrip);introColumns+=width;
  }
  introFrames++;introMaxDrawUs=max(introMaxDrawUs,uint32_t(micros()-drawStart));
  if(introColumns==480)Serial.printf("BOOT_ANIMATION_DONE direction=left_to_right duration_ms=%lu updates=%lu longest_draw_us=%lu\r\n",(unsigned long)(millis()-introStarted),(unsigned long)introFrames,(unsigned long)introMaxDrawUs);
 }
 void uiTick(){
  if(intro){animateIntro();if(introColumns<480||(int32_t)(millis()-introEnd)<0)return;intro=false;uiDirty=true;}
  if(uiDirty){
   uiDirty=false;paintedButton=-1;
   if(settings){paintSettings();if(backlightPending){digitalWrite(45,HIGH);backlightPending=false;}return;}
   if(modal){paintModal();return;}
   push(0,0,480,320,remote_main);paintStatus();paintBattery();
   for(int i=0;i<4;i++)paintButton(i,false);
  }
  if(modal||settings)return;
  if(statusIndex()!=paintedStatus)paintStatus();
  if(batteryPercent!=paintedBattery)paintBattery();
  if(visualButton!=paintedButton){
   if(paintedButton>=0)paintButton(paintedButton,false);
   paintedButton=visualButton;if(paintedButton>=0)paintButton(paintedButton);
  }
 }
 int readTouch(int &sx,int &sy){
  touchSensor.read();uint8_t n=touchSensor.touches;
  int x=touchSensor.points[0].x,y=touchSensor.points[0].y;
  if(n!=previousTouchCount||(n>0&&n<=2&&millis()-lastTouchDiagnostic>=200)){
   Serial.printf("TOUCH_RAW count=%u x=%d y=%d irq=%d\r\n",n,x,y,digitalRead(17));
   previousTouchCount=n;lastTouchDiagnostic=millis();
  }
  if(n==0)return 0;if(n>2)return -1;if(n!=1)return 2;
  if(x>=320||y>=480)return -1;
  // Match TFT_eSPI rotation 3: reflect the raw Y axis, then swap axes.
  sx=479-y;sy=x;
  if(screenFlipped){sx=479-sx;sy=319-sy;}
  return 1;
 }
 void touchTick(){
  if(intro||millis()-lastTouchPoll<20)return;lastTouchPoll=millis();int x=0,y=0;int result=readTouch(x,y);
  if(result<0){if(millis()-lastTouchGood>120){requestStop();touchWasDown=false;}return;}
  lastTouchGood=millis();
  if(result==0){touchWasDown=false;inputArmed=true;if(desired!=Direction::Stop){desired=Direction::Stop;forceControl=true;}visualButton=-1;return;}
  if(result!=1){requestStop();touchWasDown=true;return;}
  bool tap=!touchWasDown;touchWasDown=true;
  if(bootDown){requestStop();return;}
  if(settings){
   requestStop();
   if(tap&&CLOSE.contains(x,y)){settings=false;uiDirty=true;Serial.println("SETTINGS_CLOSE");return;}
   if(tap&&FLIP.contains(x,y)){
    digitalWrite(45,LOW);screenFlipped=!screenFlipped;
    lcd.setRotation(screenFlipped?1:3);prefs.putBool("flipped",screenFlipped);
    backlightPending=true;uiDirty=true;
    Serial.printf("SCREEN_FLIP flipped=%u rotation=%u\r\n",screenFlipped,screenFlipped?1:3);
   }
   if(tap&&MODE.contains(x,y)){
    twoHand=!twoHand;prefs.putBool("twohand",twoHand);uiDirty=true;
    Serial.printf("CONTROL_MODE two_hand=%u\r\n",twoHand);
   }
   return;
  }
  if(modal){
   requestStop();
   if(tap&&CLOSE.contains(x,y)){modal=false;manualPairing=false;uiDirty=true;if(!linked&&!pairPending)beginScan(false);return;}
   if(tap&&x>=74&&x<406&&y>=100&&y<238){int row=(y-100)/48;if((y-100)%48<42){int i=candidateForRow(row);if(i>=0)beginPair(candidates[i].mac,candidates[i].channel);}}
   return;
  }
  if(tap&&BATTERY.contains(x,y)){
   requestStop();if(linked)sendPacket(Kind::Control,peerMac,0,0);
   settings=true;uiDirty=true;Serial.println("SETTINGS_OPEN");return;
  }
  if(tap&&LINK.contains(x,y)){
   if(linked)sendPacket(Kind::Control,peerMac,0,0);modal=true;beginScan(true);uiDirty=true;return;
  }
  int next=-1;for(int i=0;i<4;i++)if(buttons()[i].contains(x,y)){next=i;break;}
  if(next!=visualButton){visualButton=next;Serial.printf("BUTTON index=%d x=%d y=%d linked=%u\r\n",next,x,y,linked);}
  Direction command=next>=0&&linked&&inputArmed?(Direction)next:Direction::Stop;
  if(command!=desired){desired=command;forceControl=true;}
 }
 int percentForMv(uint16_t mv){
  constexpr uint16_t volts[]={3300,3500,3600,3700,3800,3900,4000,4100,4200};
  constexpr uint8_t levels[]={0,5,10,25,45,65,80,93,100};
  if(mv<=volts[0])return 0;
  for(int i=1;i<9;i++)if(mv<=volts[i])return levels[i-1]+(mv-volts[i-1])*(levels[i]-levels[i-1])/(volts[i]-volts[i-1]);
  return 100;
 }
 void batteryTick(bool initial=false){
  if(!initial&&millis()-lastBattery<5000)return;lastBattery=millis();
  uint32_t mv=0;for(int i=0;i<8;i++)mv+=analogReadMilliVolts(9);
  mv=(mv/8)*2; // ES3C40P R14=R15, battery/2 at GPIO9.
  if(mv<2700||mv>4400){batteryMv=0;batteryPercent=-1;return;}
  batteryMv=initial||!batteryMv?mv:(batteryMv*3+mv)/4;batteryPercent=percentForMv(batteryMv);
 }
 void sleepNow(){
  requestStop();
  if(radioReady&&linked){for(int i=0;i<3;i++){uint32_t wait=millis();while(txBusy.load()&&millis()-wait<40)delay(1);sendPacket(Kind::Control,peerMac,0,0);delay(15);}}
  if(radioReady){esp_now_unregister_recv_cb();esp_now_unregister_send_cb();esp_now_deinit();radioReady=false;}
  WiFi.mode(WIFI_OFF);
  rgbLedWrite(42,0,0,0);
  if(displayReady){lcd.writecommand(0x28);lcd.writecommand(0x10);delay(120);}
  pinMode(45,OUTPUT);digitalWrite(45,LOW);pinMode(1,OUTPUT);digitalWrite(1,HIGH);pinMode(18,OUTPUT);digitalWrite(18,LOW);
  Serial.println("POWER_OFF deep_sleep=1 wake=BOOT_long_press");Serial.flush();
  while(digitalRead(0)==LOW)delay(5); // Do not immediately re-wake on the shutdown press.
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_0,0);rtc_gpio_pullup_en(GPIO_NUM_0);rtc_gpio_pulldown_dis(GPIO_NUM_0);
  gpio_hold_en(GPIO_NUM_45);gpio_hold_en(GPIO_NUM_1);gpio_hold_en(GPIO_NUM_18);gpio_deep_sleep_hold_en();
  delay(20);esp_deep_sleep_start();
 }
 void powerTick(){
  bool down=digitalRead(0)==LOW;
  if(bootGuard){if(!down)bootGuard=false;return;}
  if(down&&!bootDown){bootDown=true;bootPressedAt=millis();requestStop();}
  if(!down)bootDown=false;
  if(bootDown&&millis()-bootPressedAt>=LONG_PRESS_MS)sleepNow();
 }
 void setup(){
  instance=this;Serial.begin(115200);
  gpio_deep_sleep_hold_dis();gpio_hold_dis(GPIO_NUM_45);gpio_hold_dis(GPIO_NUM_1);gpio_hold_dis(GPIO_NUM_18);
  rtc_gpio_deinit(GPIO_NUM_0);pinMode(0,INPUT_PULLUP);
  pinMode(45,OUTPUT);digitalWrite(45,LOW);pinMode(1,OUTPUT);digitalWrite(1,HIGH);
  rgbLedWrite(42,0,0,0);
  if(esp_sleep_get_wakeup_cause()==ESP_SLEEP_WAKEUP_EXT0){
   uint32_t start=millis();while(digitalRead(0)==LOW&&millis()-start<WAKE_HOLD_MS)delay(5);
   if(millis()-start<WAKE_HOLD_MS)sleepNow();
  }
  protocolOK=walle::selfTest();Serial.printf("PROTOCOL_SELFTEST %s packet_bytes=%u\r\n",protocolOK?"PASS":"FAIL",(unsigned)sizeof(Packet));
  prefs.begin("walle_remote",false);
  screenFlipped=prefs.getBool("flipped",false);
  twoHand=prefs.getBool("twohand",true);
  Serial.printf("BUILD %s by Azucia compiled=%s %s flipped=%u\r\n",WALLE_BUILD_REV,__DATE__,__TIME__,screenFlipped);
  haveStored=prefs.getBytesLength("peer")==6;if(haveStored)prefs.getBytes("peer",storedMac,6);
  storedChannel=prefs.getUChar("channel",6);if(storedChannel<1||storedChannel>LAST_SCAN_CHANNEL)storedChannel=6;
  // The vendor display driver enables TFT_BL inside init(). Keep the physical
  // backlight low until GRAM is cleared. Reveal only the new logo columns.
  gpio_hold_en(GPIO_NUM_45);
  lcd.init();displayReady=true;digitalWrite(45,LOW);lcd.setRotation(screenFlipped?1:3);
  lcd.fillScreen(TFT_BLACK);
  touchSensor.begin();touchSensor.setRotation(ROTATION_NORMAL);pinMode(17,INPUT_PULLUP);Wire.setTimeOut(35);
  analogReadResolution(12);analogSetPinAttenuation(9,ADC_11db);batteryTick(true);initRadio();
  delay(30);gpio_hold_dis(GPIO_NUM_45);digitalWrite(45,HIGH);
  introStarted=millis();lastIntroFrame=introStarted;introEnd=introStarted+1750;
 }
 void loop(){
  powerTick();touchTick();radioTick();batteryTick();uiTick();
  if(millis()-lastLog>=3000){lastLog=millis();Serial.printf("WALLE_REMOTE v1.4 ui=%s radio=%u linked=%u pairing=%u channel=%u found=%d battery_mv=%u battery_pct=%d command=%d uptime=%lu\r\n",intro?"intro":settings?"settings":modal?"pairing":"remote",radioReady,linked,pairPending,currentChannel,candidateCount(),batteryMv,batteryPercent,(int)desired,(unsigned long)millis());}
  delay(2);
 }
};
RemoteApp *RemoteApp::instance=nullptr;
RemoteApp remote;
}
void remoteSetup(){remote.setup();}
void remoteLoop(){remote.loop();}
