#pragma once

#include "rlbot/bot.h"

#include "sequence/sequence.h"

class BadBot : public rlbot::Bot {
//private:
	//std::shared_ptr<badbot::Sequence> currentSequence;
	
public:
	BadBot(int _index, int _team, std::string _name);
	~BadBot();
	rlbot::Controller GetOutput(rlbot::GameTickPacket gametickpacket) override;
};
