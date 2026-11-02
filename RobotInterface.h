#pragma once
#include "WallERemote/WallEProtocol.h"

// Definitions only. No robot GPIOs, motor driver, or receiver firmware yet.
namespace walle {
constexpr uint8_t ROBOT_DEFAULT_CHANNEL=6;
constexpr const char *ROBOT_DEFAULT_NAME="WALL-E";
struct RobotMotorHooks {
 void (*applyNormalizedDrive)(int16_t throttle,int16_t steering);
 void (*stopImmediately)();
};
struct RobotBinding {
 uint8_t remoteMac[6];
 uint32_t session;
 uint32_t nonce;
 uint32_t lastAcceptedSequence;
 uint32_t lastAcceptedControlMs;
 bool established;
};
inline bool robotControlMatches(const Packet &p,const uint8_t sourceMac[6],const RobotBinding &binding){
 Packet validated{};
 return decode(&p,sizeof(p),validated)&&binding.established&&p.kind==Kind::Control&&
 !memcmp(sourceMac,binding.remoteMac,6)&&p.session==binding.session&&p.nonce==binding.nonce&&
 p.leaseMs>0&&p.leaseMs<=CONTROL_LEASE_MS&&newerSequence(p.sequence,binding.lastAcceptedSequence);
}
inline bool robotControlExpired(uint32_t now,const RobotBinding &binding){
 return !binding.established||uint32_t(now-binding.lastAcceptedControlMs)>=CONTROL_LEASE_MS;
}
}
