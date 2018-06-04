#ifndef __PROTOCOL_PARSER_H
#define __PROTOCOL_PARSER_H

#include "_global.h"

extern uint8_t bMsgReady;
extern bool bDelaySend;

uint8_t ParseProtocol();
void build(uint8_t _destination, uint8_t _sensor, uint8_t _command, uint8_t _type, bool _enableAck, bool _isAck);
void Msg_NodeConfigAck(uint8_t _to, uint8_t _ncf);
void Msg_NodeConfigData(uint8_t _to);
void Msg_RequestNodeID();
void Msg_Presentation();
bool ProcessOutputCfgMsg();

// eq report message
void Msg_EQReport(uint16_t eq,uint16_t index,uint16_t current);

// current change message
void Msg_CurrentChange(uint16_t current);


#endif /* __PROTOCOL_PARSER_H */