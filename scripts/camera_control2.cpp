#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "tf/transform_listener.h"

ros::Publisher r_hip_hand_pub, l_hip_hand_pub, r_hand_shoulder_pub, l_hand_shoulder_pub;

geometry_msgs::Point r_hand, l_hand, r_hand_shoulder, l_hand_shoulder;

int main(int argc, char **argv){

    ros::init(argc, argv, "camera_control");
    ros::NodeHandle n;

    r_hip_hand_pub = n.advertise<geometry_msgs::Point>("/camera/unfiltered/rightHipHand", 1);
    l_hip_hand_pub = n.advertise<geometry_msgs::Point>("/camera/unfiltered/leftHipHand", 1);
    r_hand_shoulder_pub = n.advertise<geometry_msgs::Point>("/camera/rightHandShoulder", 1);
    l_hand_shoulder_pub = n.advertise<geometry_msgs::Point>("/camera/leftHandShoulder", 1);

    tf::TransformListener listener;

    while (n.ok()){
        tf::StampedTransform r_hip_hand_tf, l_hip_hand_tf, r_hand_shoulder_tf ,l_hand_shoulder_tf;
        try{
            listener.waitForTransform("/left_hip_1", "/left_hand_1", ros::Time::now(), ros::Duration(3.0));
            listener.waitForTransform("/right_hip_1", "/right_hand_1", ros::Time::now(), ros::Duration(3.0));
            listener.waitForTransform("/right_hand_1", "/right_shoulder_1", ros::Time::now(), ros::Duration(3.0));
            listener.waitForTransform("/left_hand_1", "/left_shoulder_1", ros::Time::now(), ros::Duration(3.0));

        //users right hand to right hip (obs mirrored)
            listener.lookupTransform("/left_hip_1", "/left_hand_1", ros::Time(0), r_hip_hand_tf);

        //users left hand to left hip (obs mirrored)
            listener.lookupTransform("/right_hip_1", "/right_hand_1", ros::Time(0), l_hip_hand_tf);

        //users right hand to right shoulder (obs mirrored)
            listener.lookupTransform("/right_hand_1", "/right_shoulder_1", ros::Time(0), l_hand_shoulder_tf);

        //users left hand to left shoulder (obs mirrored)
            listener.lookupTransform("/left_hand_1", "/left_shoulder_1", ros::Time(0), r_hand_shoulder_tf);

        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        //make msgs
        r_hand.x = -r_hip_hand_tf.getOrigin().z();           //tar du armen bort frÃ¥n kroppen gÃ¥r roboten bort frÃ¥n basen
        r_hand.y = r_hip_hand_tf.getOrigin().x();            //negera om du inte vill spegla rÃ¶relsen till anvÃ¤ndaren
        r_hand.z = r_hip_hand_tf.getOrigin().y();

        l_hand.x = -l_hip_hand_tf.getOrigin().z();           
        l_hand.y = l_hip_hand_tf.getOrigin().x();            
        l_hand.z = l_hip_hand_tf.getOrigin().y();

        r_hand_shoulder.x = r_hand_shoulder_tf.getOrigin().z();
        r_hand_shoulder.y = r_hand_shoulder_tf.getOrigin().x();
        r_hand_shoulder.z = r_hand_shoulder_tf.getOrigin().y();

        l_hand_shoulder.x = l_hand_shoulder_tf.getOrigin().z();
        l_hand_shoulder.y = l_hand_shoulder_tf.getOrigin().x();
        l_hand_shoulder.z = l_hand_shoulder_tf.getOrigin().y();


        //publish msgs
        r_hip_hand_pub.publish(r_hand);
        l_hip_hand_pub.publish(l_hand);
        r_hand_shoulder_pub.publish(r_hand_shoulder);
        l_hand_shoulder_pub.publish(l_hand_shoulder);

    }

    ros::spin();

    return 0;
}

