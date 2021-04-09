/*

#include "sequence.h"

namespace badbot {

	namespace sequences {
		KickoffSequence::KickoffSequence(std::function<rlbot::Controller(unsigned int)> ctrl)):
			running(true),
			t(0),
			ctrl(ctrl){}	

		rlbot::Controller KickoffSequence::getControl() const {
			++t;
			return ctrl(t);
		}

	}//namespace sequences
}//namespace badbot
*/