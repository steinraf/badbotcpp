#include "rlbot/rlbot_generated.h"
#include "rlbot/bot.h"
#include "rlbot/scopedrenderer.h"

#include "../badbot.h"

#include "path.h"

#include <vector>
#include <algorithm>
#include <iostream>

#include <Eigen/Dense>



using namespace Eigen;



namespace badbot {

	namespace paths {
		Bezier::Bezier(const util::Car car, const Vector3f t, const util::BallPrediction& bp) :
			target(t),
			car(car),
			bp(bp){

			float t_approx = (car.physics.location - target).norm() / car.physics.velocity.norm() + bp.predictionSlices[0].gameSeconds;

			int intercept = 0;
			for (; intercept < bp.predictionSlices.size(); ++intercept) {

				if (bp.predictionSlices[intercept].gameSeconds > t_approx) break;

			}

			if (intercept != 0) --intercept;

			Vector3f target = bp.predictionSlices[intercept].physics.location;

			float dist = (car.physics.location - target).norm();



			Vector3f p0 = car.physics.location;
			Vector3f p1 = car.physics.location + car.physics.forward() * std::min(dist, 100.0f);
			Vector3f goalVector = car.other_goal() - target;
			goalVector.z() = 0;
			goalVector.normalize();
			Vector3f p2 = target - goalVector*dist/2;
			Vector3f p3 = target;

			bool debug = true;
			if (debug && car.team == 0) {
				rlbot::ScopedRenderer renderer("BezierDebug" + car.name);
				renderer.DrawRect3D(rlbot::Color::magenta, util::convert(p0), 20, 20, 1, 1);
				renderer.DrawRect3D(rlbot::Color::magenta, util::convert(p1), 20, 20, 1, 1);
				renderer.DrawRect3D(rlbot::Color::magenta, util::convert(p2), 20, 20, 1, 1);
				renderer.DrawRect3D(rlbot::Color::magenta, util::convert(p3), 20, 20, 1, 1);
			}

			//p1 = { 0, 0, 0 };
			//p2 = { 500, 1000, 0 };
			//p3 = {-500, 1000, 0 };

			//p1 = (p0 + p3) / 2;
			//p2 = p1;

			curve = [=](float t) -> Vector3f{	return	std::pow(1 - t, 3) * p0
														+ 3 * std::pow(1 - t, 2) * t * p1
														+ 3 * (1 - t) * t * t * p2
														+ std::pow(t, 3) * p3; };

			for (size_t i = 1; i <= util::NUM_POINTS; ++i) {
				path.push_back(curve(i * 1.0f / util::NUM_POINTS));
			}
			//rlbot::ScopedRenderer renderer("Renderer" + std::to_string(car.team));

			//std::vector <const rlbot::flat::Vector3*> points;

			//points.push_back(new rlbot::flat::Vector3{ p0[0], p0[1], p0[2] });
			//points.push_back(new rlbot::flat::Vector3{ p1[0], p1[1], p1[2] });
			//points.push_back(new rlbot::flat::Vector3{ p2[0], p2[1], p2[2] });
			//points.push_back(new rlbot::flat::Vector3{ p3[0], p3[1], p3[2] });
			
			//renderer.DrawPolyLine3D(rlbot::Color::yellow, points);
		}

		
		void Bezier::getPath(std::vector<const rlbot::flat::Vector3*>& points) const {

			for (auto point : points) delete point;
			points.clear();

			//TODO IMPLEMENT ACTUAL THINGY
			//unsigned len = std::min(path.size(), util::NUM_POINTS);
			for (unsigned i = 0; i < util::NUM_POINTS; ++i) {
				points.push_back(new rlbot::flat::Vector3{ path[i][0], path[i][1], path[i][2] });
			}

		}

		rlbot::Controller Bezier::getControl() const {
			return util::optimalGroundControl(car, path);
		}
		

		
	}//namespace paths
}//namespace badbot