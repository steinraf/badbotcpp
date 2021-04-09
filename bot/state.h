#pragma once

#include <vector>

#include "rlbot/rlbot_generated.h"
#include "rlbot/packets.h"

#include "../brain/path.h"




namespace badbot {
	class State {
	protected:
		const int len_pred = 50;
	public:
		virtual void getPrediction(std::vector<const rlbot::flat::Vector3*>& points) const = 0;
		virtual bool isValid() const = 0;
		virtual rlbot::Controller getControls() const = 0;
		virtual std::string name() const = 0;
	};

	class States {
	typedef std::shared_ptr<State> SharedStatePtr;
	typedef std::vector<SharedStatePtr> SharedStateVec;
	private:
		SharedStateVec vector;
	public:
		States(SharedStateVec states) : vector(states) {};
		SharedStateVec getStates() const { return vector; };
		SharedStatePtr chooseState() const;
	};

	namespace states {

		class SimpleSteer : public State {
		private:
			badbot::paths::Bezier bezier_curve;
			const rlbot::flat::GameTickPacket& packet;
			const int index;
		public:
			SimpleSteer(const rlbot::flat::GameTickPacket& packet, const int index, const rlbot::flat::BallPrediction& bp);
			void getPrediction(std::vector<const rlbot::flat::Vector3*>& points) const override;
			bool isValid() const override;
			rlbot::Controller getControls() const override;
			std::string name() const override { return"SimpleSteer"; };
		};
		/*
		class KickOff : public State {
		private:
			const rlbot::flat::GameTickPacket& packet;
			const int index;
			const rlbot::flat::FieldInfo& fInfo;
			//std::shared_ptr<Sequence> seq;
		public:
			KickOff(const rlbot::flat::GameTickPacket& packet, 
					const int index, 
					//std::shared_ptr<Sequence> seq, 
					const rlbot::flat::FieldInfo fInfo);
			void getPrediction(std::vector<const rlbot::flat::Vector3*>& points) const override;
			bool isValid() const override;
			rlbot::Controller getControls() const override;
			std::string name() const override { return"KickOff"; };
		};*/
	}//namespace states

}//namespace badbot