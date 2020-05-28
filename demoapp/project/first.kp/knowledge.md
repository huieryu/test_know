---
authors:
- estery
- admin
created_at: 2020-05-27 00:00:00
tags:
- "\u673A\u68B0\u81C2"
- "\u6293\u53D6"
- "\u76F8\u673A\u5185\u5916\u53C2\u6570\u6807\u5B9A"
title: "\u673A\u68B0\u81C2\u6293\u53D6"
tldr: This is short description of the content and findings of the post.
updated_at: 2020-05-27 16:56:19.717354
---

随着人工智能的浪潮，基于视觉的机械臂抓取已经成为当前的一个研究热点。目前机械臂抓取的通常做法是，通过深度学习对相机的RGBD数据或点云数据处理，从而获得目标物体的空间位姿。最后，对机械臂的抓取路径进行规划完成抓取。


**主要内容**

### 手眼标定:

机械臂抓取由3部分组成：
  一、进行手眼标定，得到从相机坐标系到机械臂基座坐标系的变换矩阵；
  二、进行物体检测or抓取检测，得到目标物体在相机坐标系下的空间位置，通过手眼标定的结果转换得到目标物体在基座坐标系下的位姿；
  三、通过逆运动学求解器对目标位姿进行路径规划，机械臂运动到指定位置完成抓取。

  手眼标定是机械臂抓取的第一步，相机标定则是手眼标定的第一步。相机标定主要是标定相机的内参和畸变系数，标定结果的精度会直接影响机械臂抓取的准确性。KinectV2有开源的标定包（https://github.com/code-iai/iai_kinect2），按照相应流程即可完成对相机的标定。把标定的结果放在指定的文件夹下，启动KinectV2时会自动读取标定的结果，对RGBD图像进行校正。
手眼标定—眼在手外即相机安装在机械臂之外，与机械臂的基座相对固定，不随着机械臂的运动而运动，如图2.1所示。
       A：机械臂夹持器在机械臂基座坐标系下的位姿，from gripper to base。
       B：标定板在机械臂夹持器坐标系下的位姿，from chessboard to gripper。
       C：相机在标定板坐标系下的位姿，from camera to chessboard。
       D：相机在机械臂基座坐标系下的位姿，from camera to base。
手眼标定的最终结果是得到D—从相机坐标系到机械臂基座坐标系的变换矩阵，D=A*B*C。机械臂启动时会自动发布各关节的tf变换，通过监听tf树可以得到A。C是相机的外参，可以通过OpenCV的函数计算得到。由于标定板是被夹持器随意夹住的，故B未知。因此，只要得到B的变换，那么相机在机械臂基座坐标系下的位姿D自然也就得到了。

  首先，对B进行求解。如图2.2所示，我们让机械臂移动两个位置，由于标定板相对于夹持器是固定的,这是一个典型的AX=XB问题，有四种不同的解法，相应的论文见附件。然后，我们需要实时计算相机的外参（即C）并将其发布， OpenCV中有用于计算外参的相关函数。cv::findChessboardCorners函数可以确定图像中棋盘格各个角点的像素坐标， cv::cornerSubPix函数对检测到的角点作进一步的优化计算，使角点的精度达到亚像素级别，cv::solvePnP函数通过标定板坐标系下角点的空间坐标，图片坐标系下各角点的亚像素坐标以及相机的内参和畸变系数计算得到从标定板坐标系到相机坐标系的变换矩阵即，对应函数的官方解释详见（https://docs.opencv.org/3.3.1/d9/d0c/group__calib3d.html）。通过OpenCV，我们用函数实现了用对偶四元数求解手眼标定。最后，我们需要对手眼标定结果进行验证，这里分成两步。第一步，对于AX=XB，我们可以通过计算*A*X的误差来验证求得的X是否准确；第二步，发布从相机坐标系到机械臂基座坐标系的变换矩阵D，启动jaco臂和KinectV2，在rviz中添加tf和PointCloud2插件并订阅实时的点云，如果仿真的jaco臂与点云中真实的jaco臂基本重合，我们则认为标定的精度达到了要求，手眼标定结束。

### 物体检测　or　抓取检测

  机械臂抓取的第二步是对目标物体的检测。YOLO V3是目前最为实用的物体检测系统，通过深度卷积神经网络学到的特征来进行对象检测。在ROS下有开源的程序包（https://github.com/leggedrobotics/darknet_ros）。
  通过COCO数据集训练的YOLO V3可以检测出80类物体，如图2.3所示。当然我们也可以训练自己的数据集。YOLO V3在GTX 1080GPU可以达到30fps的实时效果。关于YOLO的相关论文见附件。
  物体检测确定抓取的目标物体，接下来要对目标物体进行抓取检测。目前我们选取物体框的中心点作为抓取点。在进一步的试验中，我们会加入对物体的抓取检测，如图2.4所示。通过深度学习对目标物体进行抓取检测，找出一个可行的抓取框。其中 (x,y) is the center of the rectangle, \theta is the orientation of the rectangle to the horizontal axis of the image, h and w are the dimensions (height and width) of the rectangle. 结合抓取框和点云便可以转换得到一个可行的抓取位姿实施抓取。目前已初步实现对物体的抓取检测，但物体检测和抓取检测两者并没有完美的结合起来，接下来会尽快实现。物体检测和抓取检测从来都是两回事，虽然都有框，但物体检测框的是物体，抓取检测框的是一个可行的抓取位置。深度学习检测物体，学习的是物体的特征；检测抓取框，学习的是物体局部的信息，即抓取位置的特征。虽然现在的物体检测已经达到了很好的效果，但如何通过一个神经网络既能检测出物体框又检测出物体的抓取框还是一个很具挑战性的难题。不过先通过物体检测找到目标物体，然后把包含目标物体的框送进抓取检测的神经网络得到抓取框或许是一个不错的方法。相关内容可参考（http://www.doc88.com/p-9813547425248.html）。
### 抓取

  确定目标物体，选取物体框的中心点作为抓取点。抓取需要给机械臂一个合适的抓取位姿（相对于机械臂基座坐标系），我们需要从物体框的中心点还原出一个可行的抓取位姿（3D position和3D orientation）。由于Kinect V2发布的点云和RGB图中的像素点是一一对应的。因此，直接对中心点的点云进行读取，即可得到抓取点在相机坐标系下的三维坐标。通过手眼标定的结果可将其转换到基座下，即得到抓取点在基座下的位置。从物体检测中，我们可以确定抓取点的位置，但并没有得到欧拉角信息。目前我们设定抓取位姿的四元数为（1 0 0 0），即机械臂是自上而下对目标物体进行抓取的。
  Jaco臂在Ubuntu16.04下的软件包下载地址为（https://github.com/CNURobotics/kinova-ros/tree/kinetic_devel）。


### 部分实现代码：
```c++
    void PickPlace::evaluate_plan(moveit::planning_interface::MoveGroup &group)
{
    bool replan = true;
    int count = 0;

    moveit::planning_interface::MoveGroup::Plan my_plan;

    while (replan == true && ros::ok())
    {
        // reset flag for replan
        count = 0;
        result_ = false;

        // try to find a success plan.
        double plan_time;
        while (result_ == false && count < 5)
        {
            count++;
            plan_time = 20+count*10;
            ROS_INFO("Setting plan time to %f sec", plan_time);
            group.setPlanningTime(plan_time);
            result_ = group.plan(my_plan);
            std::cout << "at attemp: " << count << std::endl;
            ros::WallDuration(0.1).sleep();
        }

        // found a plan
        if (result_ == true)
        {
            std::cout << "plan success at attemp: " << count << std::endl;

            replan = false;
            std::cout << "please input e to execute the plan, r to replan, others to skip: ";
            std::cin >> pause_;
            ros::WallDuration(0.5).sleep();
            if (pause_ == "r" || pause_ == "R" )
            {
                replan = true;
            }
            else
            {
                replan = false;
            }
        }
        else // not found
        {
            std::cout << "Exit since plan failed until reach maximum attemp: " << count << std::endl;
            replan = false;
            break;
        }
    }

    if(result_ == true)
    {
        if (pause_ == "e" || pause_ == "E")
        {
            group.execute(my_plan);
        }
    }
    ros::WallDuration(1.0).sleep();
}


bool PickPlace::my_pick()
{
    clear_workscene();
    ros::WallDuration(1.0).sleep();
    build_workscene();
    ros::WallDuration(1.0).sleep();

    ROS_INFO_STREAM("Press any key to send robot to home position ...");
    std::cin >> pause_;
     group_->clearPathConstraints();
    group_->setNamedTarget("Home");
    evaluate_plan(*group_);

    ros::WallDuration(1.0).sleep();
    gripper_group_->setNamedTarget("Open");
    gripper_group_->move();

    ///////////////////////////////////////////////////////////
    //// joint space without obstacle
    ///////////////////////////////////////////////////////////

    ROS_INFO_STREAM("Joint space motion planing without obstacle");
    ROS_INFO_STREAM("Demonstrates moving robot from one joint position to another");
    ROS_INFO_STREAM("Planning to go to start pose ...");
    group_->setJointValueTarget(start_joint_);
    evaluate_plan(*group_);

    ROS_INFO_STREAM("Planning to go to pre-grasp joint pose ...");
    group_->setJointValueTarget(pregrasp_joint_);
    evaluate_plan(*group_);

    ROS_INFO_STREAM("Approaching grasp position ...");
    group_->setJointValueTarget(grasp_joint_);
    evaluate_plan(*group_);

    ROS_INFO_STREAM("Grasping ...");
    gripper_action(0.75*FINGER_MAX); // partially close

    ROS_INFO_STREAM("Planning to go to retract position ...");
    group_->setJointValueTarget(postgrasp_joint_);
    evaluate_plan(*group_);

    ROS_INFO_STREAM("Planning to go to start position ...");
    group_->setJointValueTarget(start_joint_);
    evaluate_plan(*group_);

    ROS_INFO_STREAM("Releasing ...");
    gripper_action(0.0); // full open

```


    <ggplot: (280771265)>
