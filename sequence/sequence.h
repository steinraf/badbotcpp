
/*
#pragma once


#include <vector>

#include "rlbot/controller.h"

namespace badbot {

	class Sequence {
	public:
		virtual bool isRunning() const = 0;
		virtual rlbot::Controller getControl() const = 0;
	};
	namespace sequences {
		class KickoffSequence : public Sequence {
		private:
			bool running;
			mutable unsigned int t;
			std::function<rlbot::Controller(unsigned int)> ctrl;
		public:
			KickoffSequence(std::function<rlbot::Controller(unsigned int)> ctrl);
			bool isRunning() const override {return running;};
			rlbot::Controller getControl() const override;
		};
	}
}
*/