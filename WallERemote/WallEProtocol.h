#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

namespace walle {
constexpr uint32_t MAGIC=0x57414C45;
constexpr uint8_t VERSION=1;
constexpr uint16_t CONTROL_PERIOD_MS=50;
constexpr uint16_t CONTROL_LEASE_MS=250;
constexpr uint16_t ROBOT_HEARTBEAT_MS=200;
constexpr uint16_t REMOTE_LINK_TIMEOUT_MS=1000;
constexpr int16_t DRIVE_LIMIT=1000;
enum class Kind:uint8_t { Discover=1,Offer=2,PairRequest=3,PairAck=4,Control=5,Heartbeat=6 };
#pragma pack(push,1)
struct Packet {
 uint32_t magic;
 uint8_t version;
 Kind kind;
 uint16_t bytes;
 uint32_t sequence;
 uint32_t session;
 uint32_t nonce;
 int16_t throttle;
 int16_t steering;
 uint16_t batteryMv;
 uint16_t leaseMs;
 char name[16];
 uint32_t crc;
};
#pragma pack(pop)
static_assert(sizeof(Packet)==48,"Protocol packet must be 48 bytes");
inline uint32_t crc32(const void *data,size_t length){
 uint32_t crc=0xffffffffu;const uint8_t *p=(const uint8_t*)data;
 while(length--){crc^=*p++;for(int i=0;i<8;i++)crc=(crc>>1)^((crc&1)?0xedb88320u:0);}
 return ~crc;
}
inline void seal(Packet &p){p.magic=MAGIC;p.version=VERSION;p.bytes=sizeof(Packet);p.crc=crc32(&p,offsetof(Packet,crc));}
inline bool decode(const void *data,size_t length,Packet &p){
 if(length!=sizeof(Packet))return false;memcpy(&p,data,sizeof(p));
 uint8_t kind=(uint8_t)p.kind;
 return p.magic==MAGIC&&p.version==VERSION&&p.bytes==sizeof(Packet)&&kind>=1&&kind<=6&&
 p.throttle>=-DRIVE_LIMIT&&p.throttle<=DRIVE_LIMIT&&p.steering>=-DRIVE_LIMIT&&p.steering<=DRIVE_LIMIT&&
 p.crc==crc32(&p,offsetof(Packet,crc));
}
inline bool newerSequence(uint32_t next,uint32_t previous){return (int32_t)(next-previous)>0;}
enum class Direction:int8_t { Stop=-1,Forward=0,Backward=1,Left=2,Right=3 };
inline void axes(Direction d,int16_t &throttle,int16_t &steering){
 throttle=0;steering=0;
 switch(d){case Direction::Forward:throttle=DRIVE_LIMIT;break;case Direction::Backward:throttle=-DRIVE_LIMIT;break;
 case Direction::Left:steering=-DRIVE_LIMIT;break;case Direction::Right:steering=DRIVE_LIMIT;break;default:break;}
}
inline bool selfTest(){
 if(crc32("123456789",9)!=0xcbf43926u)return false;
 Packet p{};p.kind=Kind::Control;p.throttle=1000;p.leaseMs=CONTROL_LEASE_MS;seal(p);Packet read{};
 if(!decode(&p,sizeof(p),read)||decode(&p,sizeof(p)-1,read))return false;
 p.throttle=-1000;if(decode(&p,sizeof(p),read))return false;
 p.throttle=1001;seal(p);if(decode(&p,sizeof(p),read))return false;
 int16_t t,s;axes(Direction::Stop,t,s);if(t||s)return false;
 axes(Direction::Left,t,s);if(t||s!=-1000)return false;
  axes(Direction::Right,t,s);if(t||s!=1000)return false;
  axes(Direction::Forward,t,s);if(t!=1000||s)return false;
  axes(Direction::Backward,t,s);if(t!=-1000||s)return false;
 return newerSequence(1,0xffffffffu)&&!newerSequence(8,8)&&!newerSequence(7,8);
}
} // namespace walle
