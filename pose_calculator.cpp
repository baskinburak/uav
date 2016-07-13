#include <ros/ros.h>
#include <uav/Done.h>
#include <uav/UAVPose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <stdlib.h>
#include <vector>

using namespace std;

bool isEqual(double a,double b){
	return abs(a-b)>0.01 ? false:true;
}
class Linked{
	private:
		struct Node{
			double value;
			Node* next;
			Node* prev;
		};
		Node* head;
		Node* tail;
		int size;
		int maxsize;
		double sum;
	public:
		Linked(){	

		}
		void SetLinked(){
			size = 0;
			maxsize = 100;
			sum = 0;
			head = NULL;
			tail = NULL;
		}
		
		~Linked(){
			Node * temp;
			Node * helper;
			temp = head;
			head = NULL;
			tail = NULL;
			if(temp){
				while(!temp->next){
					helper = temp;
					temp = temp->next;
					temp->prev = NULL;
					helper->next = NULL;
					delete helper;
				}
				delete temp;
			}
			
			
			
		}
		void AddNodeEnd(double value){
			Node* temp = new Node;
			temp->value = value;
			temp->next = NULL;
			if(head == NULL){
				temp->prev = NULL;
				head = temp;
			}
			else if(tail->prev == NULL){
				temp->prev = head;
				head->next = temp;
			}
			else{
				temp->prev = tail;
				tail->next = temp;
			}
			tail = temp;
			sum+= value;
			size++;
		}
		
		void DelNodeStart(){
			Node* temp = head;
			sum-=temp->value;
			if(head->next == NULL){
				delete temp;
			}
			else if(head->next == tail){
				tail->prev =NULL;
				head =tail;
				temp->next =NULL;
				delete temp;
			}
			else{
				head = head->next;
				head->prev=NULL;
				temp->next = NULL;
				delete temp;
			}
			
			size--;
		}
		
		double getSum(){
			return sum;
		}
		
		int getSize(){
			return size;
		}
		
		void controlAdd(double value){
			if(size == maxsize){
				this->DelNodeStart();
				this->AddNodeEnd(value);
			}
			else
				this->AddNodeEnd(value);
		}
		
};

class PoseType{
	public:
		double x;
		double y;
		double z;
		PoseType operator=(const PoseType& pose) {
			if(this != &pose){
				this->x = pose.x;
				this->y = pose.y;
				this->z = pose.z;
			}
			return *this;
		}
};

class PidVar{
	public:
		double dt;
		double max;
		double min;
		double Kp;
		double Kd;
		double Ki;
		double pre_error;
		Linked integral_help;
		PidVar(){}
		void SetVar(double _min, double _max, double _Kp, double _Kd, double _Ki){
			min = _min;
			max = _max;
			Kp = _Kp;
			Kd = _Kd;
			Ki = _Ki;
			pre_error = 0;
			integral_help.SetLinked();
		}
		void updateIntegral(double value){
			integral_help.controlAdd(value);
		}
		double getIntegral(){
			return integral_help.getSum();
		}
};
class PoseHelp{

	private:
	
		bool desired_updated;
		bool desired_achived;
		bool desired_set;
		bool current_set;
		PoseType current_position;
		PoseType current_angular;
		PoseType desired_position;
		PoseType desired_angular;
		PidVar pidx;
		PidVar pidy;
		PidVar pidz;
		
	public:
	
		PoseHelp(){
			desired_updated = false;
			desired_achived = false;
			desired_set = false;
			current_set = false;
			current_angular.x = 0;
			current_angular.y = 0;
			current_angular.z = 0;
			desired_angular.x = 0;
			desired_angular.y = 0;
			desired_angular.z = 0;
			double Kd = 10;
			double Kp = 1;
			double Ki = 1;
			double min = -5;
			double max = 5;
			pidx.SetVar(min,max,Kp,Kd,Ki);
			pidy.SetVar(min,max,Kp,Kd,Ki);
			pidz.SetVar(min,max,Kp,Kd,Ki);
		}
		
		void CurrentUpdate(double x,double y,double z){
			current_position.x = x;
			current_position.y = y;
			current_position.z = z;
			current_set = true;
		}
		
		void DesiredUpdate(double x,double y, double z){
			if((desired_position.x == x) && (desired_position.y = y) && (desired_position.z = z))
				return;
			desired_position.x = x;
			desired_position.y = y;
			desired_position.z = z;
			desired_set = true;
			desired_updated = true;
			desired_achived = false;
		}
		
		bool getDesiredSet(){
			return desired_set;
		}
		
		bool getCurrentSet(){
			return current_set;
		}
		
		bool shouldWait(){
			return (!desired_updated) && (desired_achived);
		}
		
		PoseType getDesiredPosition(){
			return desired_position;
		}
		
		PoseType getCurrentPosition(){
			return current_position;
		}
		
		float getxDifference(){
			if(isEqual(desired_position.x, current_position.x))
				return 0;
			else
				return (desired_position.x - current_position.x);
		}
		
		float getyDifference(){
			if(isEqual(desired_position.y, current_position.y))
				return 0;
			else
				return (desired_position.y - current_position.y);
	
		}
		
		float getzDifference(){
		if(isEqual(desired_position.z, current_position.z))
				return 0;
			else
				return (desired_position.z - current_position.z);
		}
		
		void setDesiredAchived(){
			desired_achived = true;
		}
		double pidXCalculate(){
			double error = desired_position.x-current_position.x;
			double propor = error*pidx.Kp;
			double deriv = (error-pidx.pre_error)*pidx.Kd;
			pidx.updateIntegral(error);
			double integ = (pidx.getIntegral())*pidx.Ki;
			pidx.pre_error = error;
			return propor + deriv + integ;
		}
		double pidYCalculate(){
			double error = desired_position.y-current_position.y;
			double propor = error*pidy.Kp;
			double deriv = (error-pidy.pre_error)*pidy.Kd;
			pidy.updateIntegral(error);
			double integ = (pidy.getIntegral())*pidy.Ki;
			pidy.pre_error = error;
			return propor + deriv + integ;
		}
		double pidZCalculate(){
			double error = desired_position.z-current_position.z;
			double propor = error*pidz.Kp;
			double deriv = (error-pidz.pre_error)*pidz.Kd;
			pidz.updateIntegral(error);
			double integ = (pidz.getIntegral())*pidz.Ki;
			pidz.pre_error = error;
			return propor + deriv + integ;
		}
		
		
};

PoseHelp pose_handle;

ros::Publisher pub_vel;
ros::Publisher pub_done;


//Updates the current Pose
void currentPose(const geometry_msgs::PoseStamped &msg){
	pose_handle.CurrentUpdate(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
	geometry_msgs::Twist pub_msg;
	if((pose_handle.getDesiredSet() == false) || (pose_handle.getCurrentSet() == false))
		return;
	if(pose_handle.shouldWait() == false){
		double x = pose_handle.getxDifference();
		double y = pose_handle.getyDifference();
		double z = pose_handle.getzDifference();
		// if UAV is in the correct position
		if( (x== 0) && (y==0) && (z==0)){
			PoseType desired_position = pose_handle.getDesiredPosition();
			pose_handle.setDesiredAchived();
			uav::Done done_send;
			done_send.commandDone = true;
			done_send.position.x = desired_position.x;
			done_send.position.y = desired_position.y;
			done_send.position.z = desired_position.z;
			done_send.orientation.x = 0;
			done_send.orientation.y = 0;
			done_send.orientation.z = 0;
			pub_done.publish(done_send);
		}
		else{
			pub_msg.linear.x = x;
			pub_msg.linear.y = y;
			pub_msg.linear.z = z;
			pub_vel.publish(pub_msg);
		}
		 
	}
	
		
}

//Updates the desired Pose
void desiredPose(const uav::UAVPose& msg){
	pose_handle.DesiredUpdate(msg.position.x, msg.position.y, msg.position.z);
}

//Topic:CommandDone Message:Done bool x,y,z, ox,oy,oz
int main(int argc,char** argv){
	ros::init(argc,argv,"pose_calculator");
	ros::NodeHandle nh;
	pub_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	pub_done = nh.advertise<uav::Done>("CommandDone",1000);
	ros::Subscriber sub_current = nh.subscribe("ground_truth_to_tf/pose",1000,&currentPose); 
	ros::Subscriber sub_desired = nh.subscribe("DesiredPose",1000,&desiredPose);
	while(ros::ok()){
		ros::spinOnce();
		
	}
	return 0;
}
