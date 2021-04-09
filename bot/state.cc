#include "state.h"

#include <cassert>
#include <cstdlib>
#include <iostream>

#include "rlbot/rlbot_generated.h"



namespace badbot {


	std::shared_ptr<State> States::chooseState() const{
		for (auto state : vector) {
			if (state->isValid()) return state;
		}
		assert(false && "Found no valid state.");
	};

	namespace states {

		SimpleSteer::SimpleSteer(const rlbot::flat::GameTickPacket& packet, const int index, const rlbot::flat::BallPrediction &bp) :
			packet(packet),
			bezier_curve(*packet.players()->Get(index),
				util::convert(*packet.ball()->physics()->location()),
				bp),
			index(index){}

		void SimpleSteer::getPrediction(std::vector<const rlbot::flat::Vector3*>& points) const {
			bezier_curve.getPath(points);
		}
		bool SimpleSteer::isValid() const { return true; }
		rlbot::Controller SimpleSteer::getControls() const { 
			return bezier_curve.getControl(); 
		}

		/*

		KickOff::KickOff(	const rlbot::flat::GameTickPacket& packet,
							const int index,
							std::shared_ptr<Sequence> seq,
							const rlbot::flat::FieldInfo fInfo) :
			packet(packet),
			fInfo(fInfo),
			index(index),
			seq(seq){}

		void KickOff::getPrediction(std::vector<const rlbot::flat::Vector3*>& points) const {
			seq.getPath(points);
		}
		bool KickOff::isValid() const { 
			Vector3f vec = util::convert(*packet.ball()->physics()->location());
			return (vec.x() == vec.y() == 0);
		}
		rlbot::Controller KickOff::getControls() const {
			return seq.getControl();
		}

		*/

	}//namespace states

}//namespace badbot