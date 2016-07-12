#include <ros/ros.h>
#include <ros/message_event.h>
#include "rapidxml.hpp"
#include "rapidxml_utils.hpp"
#include <string>
#include <vector>
#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <streambuf>
#include <iostream>
#include <map>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <uav/UAVPose.h>
#include <uav/Done.h>

std::mutex debug_print_mutex;

class GLO {
	public:
		std::mutex mtx; // mutex for GLOBALS object.
		std::mutex action_cnd_uniq_lck_mtx; // used for waiting actions cv.used as a parameter inside unique lock.
		std::condition_variable action_cnd; // signaled when an action finishes.
		std::vector<int> action_list;
		std::map<int, bool> completed_actions;
		std::vector<std::thread*> thread_refs;
		std::vector<std::string> uav_names;
		std::map<std::string, bool> uav_action_done; // stores whether current action of uav is completed.
		GLO() { }
} GLOBALS;



class UAVException : public std::exception {
	private:
		std::string _message;
	public:
		UAVException(std::string m):_message(m) { }
		UAVException(char* m):_message(m) { }
		UAVException(const char* m):_message(m) { }
		virtual const char* what() const throw() {
			return (this->_message).c_str();
		}

		virtual ~UAVException() throw() {
			
		}
};


class Pose {
	private:
		double _pos_x;
		double _pos_y;
		double _pos_z;
		double _ori_x;
		double _ori_y;
		double _ori_z;

	public:
		Pose() {
		}
		Pose(double x, double y, double z, double ox, double oy, double oz):_pos_x(x), _pos_y(y), _pos_z(z), _ori_x(ox), _ori_y(oy), _ori_z(oz) { }
		Pose(const Pose& pose) {
			this->_pos_x = pose._pos_x;
			this->_pos_y = pose._pos_y;
			this->_pos_z = pose._pos_z;
			this->_ori_x = pose._ori_x;
			this->_ori_y = pose._ori_y;
			this->_ori_z = pose._ori_z;
		}
		Pose operator=(const Pose& pose) {
			if(this != &pose) {
				this->_pos_x = pose._pos_x;
				this->_pos_y = pose._pos_y;
				this->_pos_z = pose._pos_z;
				this->_ori_x = pose._ori_x;
				this->_ori_x = pose._ori_x;
				this->_ori_x = pose._ori_x;
			}
			return *this;
		}
		double get_pos_x() { return this->_pos_x; }
		double get_pos_y() { return this->_pos_y; }
		double get_pos_z() { return this->_pos_z; }
		double get_ori_x() { return this->_ori_x; }
		double get_ori_y() { return this->_ori_y; }
		double get_ori_z() { return this->_ori_z; }

		void set_pos_x(double val) { this->_pos_x = val; }
		void set_pos_y(double val) { this->_pos_y = val; }
		void set_pos_z(double val) { this->_pos_z = val; }
		void set_ori_x(double val) { this->_ori_x = val; }
		void set_ori_y(double val) { this->_ori_y = val; }
		void set_ori_z(double val) { this->_ori_z = val; }
};

class Wait {
	private:
		std::vector<int> _for;
		std::vector<int> _except;
	public:
		void add_for(int val) {
			if((this->_except).size() != 0) {
				throw UAVException("{XML ERROR} <for> and <except> are mutually exclusive.");
			}
			this->_for.push_back(val);
		}

		void add_except(int val) {
			if((this->_for).size() != 0) {
				throw UAVException("{XML ERROR} <for> and <except> are mutually exclusive.");
			}
			this->_except.push_back(val);
		}

		Wait operator=(Wait& wait) {
			if(this != &wait) {
				this->_for = wait._for;
				this->_except = wait._except;
			}
			return *this;
		}

		Wait() {
		}

		int get_for(int idx) {
			if(idx<(this->_for).size())
				return (this->_for)[idx];
			return -1;
		}

		int get_except(int idx) {
			if(idx<(this->_except).size())
				return (this->_except)[idx];
			return -1;
		}

		bool has_except(int except) { // return if this has specific except.
			for(int i=0; i<(this->_except).size(); i++) {
				if(except == (this->_except)[i])
					return true;
			}
			return false;
		}

		bool has_except() { // returns if this has any except.
			return (this->_except).size() > 0;
		}
};

class Action {
	private:
		int _id;
		Pose _goto_pose;
		Wait _wait;
	public:

		Action* next;

		Action(Action& action) {
			this->_id = action._id;
			this->_goto_pose = action._goto_pose;
			this->_wait = action._wait;
		}

		Action operator=(Action& action) {
			if(this != &action) {
				this->_id = action._id;
				this->_goto_pose = action._goto_pose;
				this->_wait = action._wait;
			}
			return *this;
		}

		Action(int i):_id(i) { 
			next = NULL;
		}

		void set_goto_pose(Pose pose) {
			this->_goto_pose = pose;
		}

		void add_wait_for(int val) {
			(this->_wait).add_for(val);
		}

		void add_wait_except(int val) {
			(this->_wait).add_except(val);
		}

		int get_wait_for(int idx) {
			return (this->_wait).get_for(idx);
		}

		int get_wait_except(int idx) {
			return (this->_wait).get_except(idx);
		}

		bool has_wait_except(int except) {
			return (this->_wait).has_except(except);
		}

		bool has_wait_except() {
			return (this->_wait).has_except();
		}

		int get_id() {
			return this->_id;
		}

		uav::UAVPose get_UAVPose() {
			uav::UAVPose pose;

			pose.position.x = (this->_goto_pose).get_pos_x();
			pose.position.y = (this->_goto_pose).get_pos_y();
			pose.position.z = (this->_goto_pose).get_pos_z();
			pose.orientation.x = (this->_goto_pose).get_ori_x();
			pose.orientation.y = (this->_goto_pose).get_ori_y();
			pose.orientation.z = (this->_goto_pose).get_ori_z();

			return pose;
		}
};

class ActionList {
	private:
		Action* _head;
		Action* _tail;
	public:
		ActionList() {
			_head = NULL;
			_tail = NULL;
		}

		void add_action(Action& action) {
			if(this->_head == NULL) {
				action.next = NULL;
				this->_head = &action;
				this->_tail = &action;
			} else {
				this->_tail->next = &action;
				this->_tail = &action;
				this->_tail->next = NULL;
			}
		}

		Action* get_head() { //also removes it.
			Action* ret = this->_head;
			if(this->_head != NULL) {
				this->_head = this->_head->next;
				if(this->_head == NULL) {
					this->_tail = NULL;
				}
			}
			return ret;
		}

		bool has_action() {
			return this->_head != NULL;
		}


};

class UAV {
	private:
		ActionList _action_list;
		std::string _name;
		Pose _spawn_point;
		Action* _current_action;
	public:
		UAV(std::string n):_name(n) {
			_current_action = NULL;
		}

		bool has_action_left() {
			return (this->_action_list).has_action();
		}

		void set_spawn_pose(double x, double y, double z, double ox, double oy, double oz) {
			_spawn_point.set_pos_x(x);
			_spawn_point.set_pos_y(y);
			_spawn_point.set_pos_z(z);
			_spawn_point.set_ori_x(ox);
			_spawn_point.set_ori_y(oy);
			_spawn_point.set_ori_z(oz);
		}

		void set_spawn_pose(const Pose &pose) {
			this->_spawn_point = pose;
		}

		void add_action(Action& action) {
			Action* act = new Action(action);
			(this->_action_list).add_action(*act);
		}

		std::string get_name() {
			return this->_name;
		}

		Action& get_action() {
			this->_current_action = (this->_action_list).get_head()
			return *(this->current_action);
		}
};

bool action_can_proceed(Action& action) {
	bool has_for = false;
	GLOBALS.mtx.lock();
	for(int i=0, action_id=action.get_wait_for(i); action_id!=-1; action_id=action.get_wait_for(++i)) {
		has_for = true;
		if(!GLOBALS.completed_actions[action_id]) {
			GLOBALS.mtx.unlock();
			return false;
		}
	}

	if(!has_for && action.has_wait_except()) {
		for(int i=0; i<GLOBALS.action_list.size(); i++) {
			if(!action.has_wait_except(GLOBALS.action_list[i]) && !GLOBALS.completed_actions[GLOBALS.action_list[i]]) {
				GLOBALS.mtx.unlock();
				return false;
			}
		}
	}
	GLOBALS.mtx.unlock();
	return true;
}

void answer_received(const ros::MessageEvent<uav::Done const>& event) {
	const uav::Done message = event.getMessage();
	
}

void drive_UAV(UAV& uav, ros::NodeHandle& nh) {

	ros::Publisher publisher = nh.advertise<uav::UAVPose>(uav.get_name() + "/DesiredPose", 1000);
	ros::Subscriber subscriber = nh.subscribe(uav.get_name() + "/CommandDone", 1000, &answer_received);

	debug_print_mutex.lock();
	std::cout << "Starting " << uav.get_name() << std::endl;
	debug_print_mutex.unlock();

	while(uav.has_action_left()) {

		debug_print_mutex.lock();
		std::cout << uav.get_name() << " has action left" << std::endl;
		debug_print_mutex.unlock();

		Action action = uav.get_action();
		// send desired pose to DesiredPose
		// CommandDone // done true? false, x, y, z, ox, oy, oz

		GLOBALS.mtx.lock();

		std::unique_lock<std::mutex> ulck(GLOBALS.action_cnd_uniq_lck_mtx);
		GLOBALS.mtx.unlock();
		while(!action_can_proceed(action)) {
			debug_print_mutex.lock();
			std::cout << action.get_id() << " cannot proceed." << std::endl;
			debug_print_mutex.unlock();
			GLOBALS.action_cnd.wait(ulck);
		}
		ulck.unlock();
		//now the action can proceed.
		debug_print_mutex.lock();
		std::cout << action.get_id() << " can proceed" << std::endl;
		debug_print_mutex.unlock();

		ros::Rate try_rate(1);

		while(ros::ok() && !GLOBALS.uav_action_done(uav.get_name())) {
			uav::UAVPose pose = action.get_UAVPose();
			publisher.publish(pose);
			try_rate.sleep();
		}

	}
}


Pose read_pose(rapidxml::xml_node<>* node) {

	double s_x, s_y, s_z, s_ox, s_oy, s_oz;

	rapidxml::xml_node<> *pose_node = node->first_node("pos_x");
	if(!pose_node) {
		throw UAVException("{XML ERROR} No x coordinate specified");
	}
	s_x = std::stod(pose_node->value());

	pose_node = node->first_node("pos_y");
	if(!pose_node) {
		throw UAVException("{XML ERROR} No y coordinate specified");
	}
	s_y = std::stod(pose_node->value());

	pose_node = node->first_node("pos_z");
	if(!pose_node) {
		throw UAVException("{XML ERROR} No z coordinate specified");
	}
	s_z = std::stod(pose_node->value());

	pose_node = node->first_node("ori_x");
	if(!pose_node) {
		throw UAVException("{XML ERROR} No x orientation specified");
	}
	s_ox = std::stod(pose_node->value());

	pose_node = node->first_node("ori_y");
	if(!pose_node) {
		throw UAVException("{XML ERROR} No y orientation specified");
	}
	s_oy = std::stod(pose_node->value());

	pose_node = node->first_node("ori_z");
	if(!pose_node) {
		throw UAVException("{XML ERROR} No z orientation specified");
	}
	s_oz = std::stod(pose_node->value());

	return Pose(s_x, s_y, s_z, s_ox, s_oy, s_oz);
}


void populate(ros::NodeHandle& nh) {
	rapidxml::file<> file("/home/baskin/KOVAN/src/uav/uav_src/sample.xml");
	rapidxml::xml_document<> xml_doc;
	xml_doc.parse<rapidxml::parse_default>(file.data());
	rapidxml::xml_node<> *plan_node = xml_doc.first_node("plan");
	for(rapidxml::xml_node<> *uav_node = plan_node->first_node("uav"); uav_node; uav_node = uav_node->next_sibling("uav")) {
		std::string id = "";
		for(rapidxml::xml_attribute<> *uav_attr = uav_node->first_attribute(); uav_attr; uav_attr = uav_attr->next_attribute()) {
			if(uav_attr->name() == std::string("id")) {
				id = uav_attr->value();
				break;
			}
		}

		if(id == "") {
			throw UAVException("{XML ERROR} No ID specified for the UAV");
		}

		std::string name = "uav" + id;

		GLOBALS.mtx.lock();
		if(find(GLOBALS.uav_names.begin(), GLOBALS.uav_names.end(), name) != GLOBALS.uav_names.end()) {
			throw UAVException("{XML ERROR} Duplicate UAV name " + name);
		}
		GLOBALS.uav_names.push_back(name);
		GLOBALS.mtx.unlock();

		UAV *uavp = new UAV(name);
		UAV &uav = *uavp;
		rapidxml::xml_node<> *spawn_node = uav_node->first_node("spawn");
		if(!spawn_node) {
			throw UAVException(std::string("{XML ERROR} No spawn point specified for UAV ") + name);
		}

		Pose spawn_pose = read_pose(spawn_node);

		uav.set_spawn_pose(spawn_pose);

		for(rapidxml::xml_node<> *action_node = uav_node->first_node("action"); action_node; action_node = action_node->next_sibling("action")) {
			id="";

			for(rapidxml::xml_attribute<> *id_attr = action_node->first_attribute(); id_attr; id_attr=id_attr->next_attribute()) {
				if(id_attr->name() == std::string("id")) {
					id = id_attr->value();
					break;
				}
			}

			if(id == "") {
				throw UAVException(std::string("{XML ERROR} UAV ") + name + std::string(" has an action without id attribute"));
			}

			int _id = std::stoi(id);
			if(_id < 0) {
				throw UAVException("Action IDs must not be negative");
			}


			Action action(_id);

			rapidxml::xml_node<> *goto_node = action_node->first_node("goto_pose");
			if(!goto_node) {
				throw UAVException(std::string("{XML ERROR} Action with id ") + id + std::string(" does not have <goto_pose> as child"));
			}

			Pose pose = read_pose(goto_node);
			action.set_goto_pose(pose);

			rapidxml::xml_node<> *wait_node = action_node->first_node("wait");

			if(wait_node) {
				rapidxml::xml_node<> *for_node = wait_node->first_node("for");
				if(for_node) {
					for(; for_node; for_node = for_node->next_sibling("for")) {
						action.add_wait_for(std::stoi(for_node->value()));
					}
				} else {
					rapidxml::xml_node<> *except_node = wait_node->first_node("except");
					if(except_node) {
						for(; except_node; except_node = except_node->next_sibling("except")) {
							action.add_wait_except(std::stoi(except_node->value()));
						}
					}
				}
			}
			GLOBALS.mtx.lock();
			if(find(GLOBALS.action_list.begin(), GLOBALS.action_list.end(), _id) != GLOBALS.action_list.end()) {
				throw UAVException("{XML ERROR} Duplicate action id " + std::string(id));
			}
			GLOBALS.action_list.push_back(_id);
			GLOBALS.mtx.unlock();
			uav.add_action(action);
		}
		// start uav.
		std::thread *uav_thread = new std::thread(&drive_UAV, std::ref(uav), std::ref(nh));
		GLOBALS.mtx.lock();
		GLOBALS.thread_refs.push_back(uav_thread);
		GLOBALS.mtx.unlock();
	}
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "uav_master");
	ros::NodeHandle nh;

	populate(nh);

	for(int i=0; i<GLOBALS.thread_refs.size(); i++) {
		(GLOBALS.thread_refs[i])->join();
	}
	return 0;
}

