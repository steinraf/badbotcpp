#pragma once 

#include <Eigen/Dense>

#include "rlbot/rlbot_generated.h"

#include "../badbot.h"

using namespace Eigen;



namespace util {

	const int NUM_POINTS = 50;

	

	class Rotation {
	public:
		const float pitch;
		const float yaw;
		const float roll;

		operator Vector3f() const;

		Rotation(const Vector3f& rot);
		Rotation(const rlbot::flat::Rotator& rot);
	};

	class BallPhysics {
	public:
		const Vector3f location;
		const Vector3f velocity;
		const Rotation rotation;
		const Vector3f angularVelocity;


		BallPhysics(const rlbot::flat::Physics& phys);
	};

	class Physics {
	public:
		const Vector3f location;
		const Vector3f velocity;
		const Rotation rotation;
		const Vector3f angularVelocity;

		

		Vector3f forward() const;
		Vector3f right() const;
		Vector3f up() const;

		Physics(const rlbot::flat::Physics& phys);
		Physics(const BallPhysics& bp);
	};

	

	

	class GameObject {
	public:
		//const Physics physics;
	
		//TODO replace car and player with generic GameObject

		//TRY implement throught template?

		virtual std::string getName() const = 0;
	};

	class ScoreInfo {
	public:
		const int score;
		const int goals;
		const int ownGoals;
		const int assists;
		const int saves;
		const int shots;
		const int demolitions;

		ScoreInfo(const rlbot::flat::ScoreInfo& info);
	};

	class BoxShape {
	public:
		const float length;
		const float width;
		const float height;
		BoxShape(const rlbot::flat::BoxShape& box);
	};

	class Car : public GameObject {
	public:
		const Physics physics;
		const ScoreInfo scoreInfo;
		const bool isDemolished;
		const bool hasWheelContact;
		const bool isSupersonic;
		const bool isBot;
		const bool jumped;
		const bool doubleJumped;
		const std::string name;
		const int team;
		const int boost;
		const BoxShape hitbox;
		const Vector3f hitboxOffset;


		Car(const rlbot::flat::PlayerInfo& inf);
		std::string getName() const override { return "Car"; };

		Vector3f other_goal() const;
		Vector3f own_goal() const;
	};

	class Touch {
	public:
		const std::string playerName;
		const float gameSeconds;
		const Vector3f location;
		const Vector3f normal;
		const int team;
		const int playerIndex;
		Touch(const rlbot::flat::Touch& t);
	};

	class DropShotBallInfo {
	public:
		const float absorbedForce;
		const int damageIndex;
		const float forceAccumRecent;
		DropShotBallInfo(const rlbot::flat::DropShotBallInfo& drop);
	};



	class CollisionShape {
	public:
		//TODO FIX TYPE
		const int type;
		Vector3f box = { 153, 153, 153 }; // length width height
		Vector3f sphere = { 184, 0, 0 };  // diameter
		Vector3f cylinder = {184, 30, 0}; // diameter height
		CollisionShape(const rlbot::flat::CollisionShape coll);
	};

	class Ball : public GameObject {
	public:
		const Physics physics;
		const Touch latestTouch;
		const DropShotBallInfo dropShotInfo;
		const CollisionShape shape_type;

		std::string getName() const override { return "Ball"; };

		Ball(const rlbot::flat::BallInfo& ball);
	};

	class PredictionSlice {
	public:
		const float gameSeconds;
		const BallPhysics physics;

		PredictionSlice(const rlbot::flat::PredictionSlice* slice);
	};

	class BallPrediction {
	public:
		std::vector<PredictionSlice> predictionSlices;

		BallPrediction(const rlbot::flat::BallPrediction& bp);
	};

	template<typename T>
	T map(T value, T old_min, T old_max, T new_min, T new_max);

	template<typename T>
	T clamp(T value, T min, T max);

	Vector3f convert(const rlbot::flat::Vector3& vec);
	Vector3f convert(const rlbot::flat::Rotator& vec);

	rlbot::flat::Vector3 convert(const Vector3f& vec);

	rlbot::Controller optimalGroundControl(const Car& car, const std::vector<Vector3f>& path);


}//namespace util