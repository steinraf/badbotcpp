#pragma once

#include "rlbot/rlbot_generated.h"
#include "rlbot/packets.h"
#include "rlbot/bot.h"

#include "../util/util.h"

#include <vector>

#include <Eigen/Dense>

//using namespace Eigen;




namespace badbot {


	class Path {

	public:
		//Path(const rlbot::flat::GameTickPacket& packet);
		virtual void getPath(std::vector<const rlbot::flat::Vector3*>& points) const = 0;
		virtual rlbot::Controller getControl() const = 0;
		virtual Vector3f getTarget() const = 0;
		virtual Vector3f predict(double t) const = 0;
		virtual std::string name() const = 0;

	};

	namespace paths {
		class Bezier : public Path {
		private:
			const Eigen::Vector3f target;
			const util::Car car;
			const util::BallPrediction& bp;

			std::vector<Eigen::Vector3f> path;
			std::function<Eigen::Vector3f(float)> curve;
			
		public:
			Bezier(const util::Car car, Eigen::Vector3f target, const util::BallPrediction& bp);
			void getPath(std::vector<const rlbot::flat::Vector3*>& points) const override;
			rlbot::Controller getControl() const override;
			Eigen::Vector3f getTarget() const override{ return target; }
			Eigen::Vector3f predict(double t) const override { return curve(t); }
			std::string name() const override{ return "Bezier"; }
		};

		


	}//namespace paths
} //namespace badbot
