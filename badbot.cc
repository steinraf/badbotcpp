#include "badbot.h"

#include <ctime>
#include <math.h>
#include <string>
#include <iostream>

#include "rlbot/bot.h"
#include "rlbot/color.h"
#include "rlbot/interface.h"
#include "rlbot/rlbot_generated.h"
#include "rlbot/scopedrenderer.h"
#include "rlbot/statesetting.h"

#include "bot/state.h"

#include "util/util.h"


BadBot::BadBot(int _index, int _team, std::string _name)
	: Bot(_index, _team, _name) {
	rlbot::GameState gamestate = rlbot::GameState();

	gamestate.ballState.physicsState.location = { 0, 0, 1000 };
	gamestate.ballState.physicsState.velocity = { 0, 0, 5000 };

	rlbot::CarState carstate = rlbot::CarState();
	carstate.physicsState.location = { 0, 500, 1000 };
	carstate.physicsState.velocity = { 500, 1000, 1000 };
	carstate.physicsState.angularVelocity = { 1, 2, 3 };

	carstate.boostAmount = 50;

	gamestate.carStates[_index] = carstate;

	rlbot::Interface::SetGameState(gamestate);


}

BadBot::~BadBot() {
	// Free your allocated memory here.
}

rlbot::Controller BadBot::GetOutput(rlbot::GameTickPacket gametickpacket) {
	/*
	if (currentSequence != nullptr) {
		if (currentSequence->isRunning()) {
			return currentSequence->getControl();
		}
		else currentSequence = nullptr;
	}
	*/
	const rlbot::BallPrediction ballPred = GetBallPrediction();

	const rlbot::flat::GameTickPacket& packet = *gametickpacket.getRoot();
	const rlbot::flat::BallPrediction& bp = *ballPred.getRoot();

	//const rlbot::flat::FieldInfo &fInfo = *GetFieldInfo().getRoot();

	//Add states in descending priority
	std::vector<std::shared_ptr<badbot::State>> possible_states = {
		//std::make_shared<badbot::states::KickOff>(packet, index, currentSequence, fInfo),
		std::make_shared<badbot::states::SimpleSteer>(packet, index, bp)
	};


	badbot::States states(possible_states);

	std::shared_ptr<badbot::State> currentState = states.chooseState();
	std::string state_debug = "Bot " + std::to_string(index) + "'s state: " + currentState->name();

	rlbot::ScopedRenderer renderer("Renderer" + std::to_string(index));

	renderer.DrawString2D(state_debug, rlbot::Color::green,
		rlbot::flat::Vector3{ 10, 10.0f + 50.0f * index, 0 }, 4, 4);

	std::vector<const rlbot::flat::Vector3*> points;
	currentState->getPrediction(points);
	renderer.DrawPolyLine3D(rlbot::Color::magenta, points);


	rlbot::Controller ctr = currentState->getControls();


	return ctr;

}
