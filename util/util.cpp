
#include <Eigen/Dense>
#include <cmath>
#include <iostream>

#include "rlbot/bot.h"
#include "rlbot/rlbot_generated.h"
#include "rlbot/scopedrenderer.h"

#include "util.h"




using namespace Eigen;



namespace util {

	const float M_PI = 3.14159265;

	Vector3f convert(const rlbot::flat::Vector3& vec) {
		return { vec.x(), vec.y(), vec.z() };
	}

	Vector3f convert(const rlbot::flat::Rotator& vec) {
		
		return { vec.pitch(), vec.yaw(), vec.roll() };
	}

	rlbot::flat::Vector3 convert(const Vector3f& vec) {
		return rlbot::flat::Vector3{ vec[0], vec[1], vec[2] };
	}

	Physics::Physics(const rlbot::flat::Physics& phys) :
		location(convert(*phys.location())),
		velocity(convert(*phys.velocity())),
		rotation(convert(*phys.rotation())),
		angularVelocity(convert(*phys.angularVelocity())) {}


	Physics::Physics(const BallPhysics& bp) :
		location(bp.location),
		velocity(bp.velocity),
		rotation(bp.rotation),
		angularVelocity(bp.angularVelocity) {}

	

	BallPhysics::BallPhysics(const rlbot::flat::Physics& phys) :
		location(convert(*phys.location())),
		velocity(convert(*phys.velocity())),
		rotation(Vector3f::Zero()),
		angularVelocity(convert(*phys.angularVelocity())) {}


	Vector3f Physics::forward() const {

		float	sp = sin(rotation.pitch),
				cp = cos(rotation.pitch),
				sr = sin(rotation.roll),
				cr = cos(rotation.roll),
				sy = sin(rotation.yaw),
				cy = cos(rotation.yaw);


		return { cp * cy, cp * sy, sp };
	}

	Vector3f Physics::right() const {
		//pitch roll yaw
		float	sp = sin(rotation.pitch),
				cp = cos(rotation.pitch),
				sr = sin(rotation.roll),
				cr = cos(rotation.roll),
				sy = sin(rotation.yaw),
				cy = cos(rotation.yaw);


		return { cy * sp * sr - cr * sy, sy * sp * sr + cr * cy, -cp * sr };

	}

	Vector3f Physics::up() const {

		float	sp = sin(rotation.pitch),
				cp = cos(rotation.pitch),
				sr = sin(rotation.roll),
				cr = cos(rotation.roll),
				sy = sin(rotation.yaw),
				cy = cos(rotation.yaw);

		return { -cr * cy * sp - sr * sy, -cr * sy * sp + sr * cy, cp * cr };
	}

	Rotation::operator Vector3f() const {
		return Vector3f(pitch, yaw, roll);
	}

	Rotation::Rotation(const Vector3f& rot) :
		pitch(rot[0]),
		yaw(rot[1]),
		roll(rot[2]) {
	}

	Rotation::Rotation(const rlbot::flat::Rotator& rot) :
		pitch(rot.pitch()),
		yaw(rot.yaw()),
		roll(rot.roll()) {}
		

	ScoreInfo::ScoreInfo(const rlbot::flat::ScoreInfo& inf) :
		score(inf.score()),
		goals(inf.goals()),
		ownGoals(inf.ownGoals()),
		assists(inf.assists()),
		saves(inf.saves()),
		shots(inf.shots()),
		demolitions(inf.demolitions()) {}

	BoxShape::BoxShape(const rlbot::flat::BoxShape& box) :
		length(box.length()),
		width(box.width()),
		height(box.height()) {}

	Car::Car(const rlbot::flat::PlayerInfo& info) :
		physics(*info.physics()),
		scoreInfo(*info.scoreInfo()),
		isDemolished(info.isDemolished()),
		hasWheelContact(info.hasWheelContact()),
		isSupersonic(info.isSupersonic()),
		isBot(info.isBot()),
		jumped(info.jumped()),
		doubleJumped(info.doubleJumped()),
		name((*info.name()).str()),
		team(info.team()),
		boost(info.boost()),
		hitbox(*info.hitbox()),
		hitboxOffset(convert(*info.hitboxOffset())) {
	}

	Touch::Touch(const rlbot::flat::Touch& t) :
		gameSeconds(t.gameSeconds()),
		location(convert(*t.location())),
		normal(convert(*t.normal())),
		team(t.team()),
		playerIndex(t.playerIndex()) {}

	DropShotBallInfo::DropShotBallInfo(const rlbot::flat::DropShotBallInfo& drop) :
		absorbedForce(drop.absorbedForce()),
		damageIndex(drop.damageIndex()),
		forceAccumRecent(drop.forceAccumRecent()) {}

	CollisionShape::CollisionShape(rlbot::flat::CollisionShape coll) : type(0) {} //TODO FIX TYPE

	Ball::Ball(const rlbot::flat::BallInfo& ball) :
		physics(*ball.physics()),
		latestTouch(*ball.latestTouch()),
		dropShotInfo(*ball.dropShotInfo()),
		shape_type(ball.shape_type()) {}

	PredictionSlice::PredictionSlice(const rlbot::flat::PredictionSlice* slice) :
		gameSeconds(slice->gameSeconds()),
		physics(*slice->physics()) {}

	BallPrediction::BallPrediction(const rlbot::flat::BallPrediction& bp){
		int max = bp.slices()->size();
		for (int i = 0; i < max; ++i) {
			predictionSlices.push_back(bp.slices()->Get(i));

		}
	}


	Vector3f Car::other_goal() const{
		float t = team == 1 ? 1 : -1;
		return Vector3f{ 0, -5120 * t, 92.75 };
	}

	Vector3f Car::own_goal() const{
		float t = team == 1 ? 1 : -1;
		return Vector3f{ 0,  5120 * t, 92.75 };
	}

	

	Vector3f to_local(Car c, Vector3f target) {

		//TODO MAKE ACTUAL MATRIX IMPLENTATION INSTEAD OF WHATEVER THIS IS

		
		Vector3f local = {
			(target - c.physics.location).dot(c.physics.forward()),
			(target - c.physics.location).dot(c.physics.right()),
			(target - c.physics.location).dot(c.physics.up())
		};

		
		return local;
	}

	template<typename T>
	T map(T value, T old_min, T old_max, T new_min, T new_max) {
		return (value - old_min) * (new_max - new_min) / (old_max - old_min) + new_min;
	}

	template<typename T>
	T clamp(T value, T min, T max) {
		if (value < min) return min;
		if (value > max) return max;
		return value;
	}

	rlbot::Controller optimalGroundControl(const Car& car, const std::vector<Vector3f>& path) {

		bool debug = 1;

		rlbot::Controller controller{ 0 };

		//take average of first 20% of the path (10)
		Vector3f aim = { 0, 0, 0 };
		int len = std::min((int)path.size(), util::NUM_POINTS/5);
		for (auto i = 0; i < len; ++i) aim += path[i];
		aim /= len;

		//aim = path[path.size() / 2];

		//aim = path[0];

		rlbot::ScopedRenderer renderer("OPTIMAL GROUND CONTROL " + std::to_string(car.team));

		//Vector3f local = to_local(car, aim);

		float angle = atan2(aim.y() - car.physics.location.y(), 
							aim.x() - car.physics.location.x());


		angle -= car.physics.rotation.yaw;

	

		if (angle < -M_PI) angle += 2 * M_PI;
		if (angle > M_PI) angle -= 2 * M_PI;

		float steer = angle/M_PI;

		steer *= 10;
		steer = clamp<float>(steer, -1, 1);

		//steer = steer > 0 ? 1 : -1;
		//steer = clamp<float>(steer, -1, 1);

		if (debug && car.team == 0) {
			renderer.DrawRect3D(rlbot::Color::black, convert(aim),
				20, 20, true, true);
		}


		//if steer is way too big, slow down
		
		//bool is_going_forward = ( car.physics.forward().dot(car.physics.velocity) > 0);
		/*
		if (is_going_forward && path[path.size() -1].z() > 400) {
			controller.throttle = 1 - 2 * std::abs(steer);
		}
		else {
			controller.throttle = 1;
		}*/

		controller.throttle = clamp<float>(1.3 - std::abs(steer), 0, 1);

		if (steer > 0.05) steer = 1;
		if (steer < -0.05) steer = -1;

		controller.steer = steer;

		

		if (std::abs(steer) < 0.05 && car.hasWheelContact) controller.boost = 1;


		//if car is stuck
		

		Vector3f origin = { 0, 0, 92.75 };

		// If ball is at center(kickoff)
		if ((path[path.size() - 1] - origin).norm() < 5) {
			controller.boost = 1;
			controller.steer = controller.steer > 0 ? 1 : -1;
		}//else if (car.physics.velocity.norm() < 5) controller.jump = 1;


		//jump if car is close to ball
		if ((path[path.size() - 1] - path[0]).norm() < 200 && car.hasWheelContact && path[path.size() - 1].z() < 200 && std::abs(controller.steer) < 0.1 ) {
			controller.jump = 1;
		}

		// If car is in the air
		if (!car.hasWheelContact) {
			if ((path[path.size() - 1] - path[0]).norm() < 200) {
				controller.pitch = 1;
				
				controller.jump = 1;
			}
			else {
				controller.pitch = clamp<float>(-2*car.physics.rotation.pitch, -1, 1);
				controller.yaw = clamp<float>(2*controller.steer, -1, 1);
				controller.roll = clamp<float>(-2*car.physics.rotation.roll, -1, 1);
			}
		}

		//if target is in the air slow down
		Vector3f target = path[path.size() - 1];
		target.z() = 0;

		Vector3f now = car.physics.location;
		now.z() = 0;

		float groundDist = (target - now).norm();

		//if (path[path.size() - 1].z() > 300 && groundDist < 500 && std::abs(controller.steer) < 0.1) 
			//controller.throttle = -1 * is_going_forward;


		if (debug && car.team == 0) {
			renderer.DrawString2D(	"boost: " + std::to_string(controller.boost) + "\n" +
									"handbrake: " + std::to_string(controller.handbrake) + "\n" +
									"jump: " + std::to_string(controller.jump) + "\n" +
									"steer: " + std::to_string(controller.steer) + "\n" +
									"throttle: " + std::to_string(controller.throttle) + "\n" +
									"pitch: " + std::to_string(controller.pitch) + "\n" +
									"roll: " + std::to_string(controller.roll) + "\n" +
									"yaw: " + std::to_string(controller.yaw) + "\n",
									rlbot::Color::green,
									rlbot::flat::Vector3{ 10, 200, 0 },
									2, 2);
									
									
		}

		return controller;

	}


}//namespace util