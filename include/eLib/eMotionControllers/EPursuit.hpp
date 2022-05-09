#ifndef E_PURE_PURSUIT
#define E_PURE_PURSUIT
#include "eLib/eMotionPlanners/Profilers/EPath.hpp"
#include "pros/rtos.hpp"
using namespace okapi;

namespace elib{

	class EPursuit{
	// private:
	public:
		// ----------------------
		// Parameters
		// ----------------------
		PathSegment path;
		const QLength lookAheadDist = 16_in;

		// ----------------------
		// State Recorder
		// ----------------------
		shared_ptr<okapi::OdomState> truePose;
		int targetIndex = -1;


		//------------------------------------------------
		// Functions
		//------------------------------------------------

	public: 
		// Constructor
		EPursuit(shared_ptr<OdomState> shared_pose);


		// ----------------------
		// Task API
		// ----------------------
		// This is the Task, not meant to be called by the user
		int calcPursuit(QLength& pursuitDist, QAngle& pursuitAngle, QLength& dist2Target);
		void clearPursuit();


		// ----------------------
		// User API
		// ----------------------
		void startPath(PathSegment targetPath);
		void startPath(const string& pathDir);

		int loadPath(const string& pathDir);
		int findTargetIndex();
		int findNearestIndex();


		// ----------------------
		// Status Monitor
		// ----------------------
		bool reachedPathEnd();
		int getPathSize();
		int getPathEnd();
	};
}


#endif