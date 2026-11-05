%   arm_interface.m
%   Interface to a simulated robot in CoppeliaSim simulator
%   2026/09/16
%   % Copyright (C) 2026
%   Author: Luís Louro, luislouro@algoritmi.uminho.pt
%           Estela Bicho, estela.bicho@dei.uminho.pt

classdef arm_interface < handle
    properties (Access = private)
        client
        sim

        RobotHandle  % handle for the robot object
        jointHandle  % 1,2,...,4 handle for each of the joints of the manipulator
        HandHandle   % handle for the hand object 
        
        ROS_WORKING  
    end

    properties
        number_of_joints  % variable indicating the number of joints of the manipulator arm
        robot_name        % variable indicating the name of the handler in CoppeliaSim
        MinJointPos       % vector indicating the minimum position allowed for the joint i
        MaxJointPos       % vector indicating the maximun position allowed for the joint i
    end

    methods
        % Classe constructor
        function [obj,error] = arm_interface(sim_obj,robot_name,ROSWORKING)
            obj.robot_name = robot_name;
            obj.ROS_WORKING = ROSWORKING;
            error = 0;

            obj.client = sim_obj.get_connection();
            obj.sim = obj.client.require('sim');

            %% Get objects handles

            %Get Arm Handle
            obj.RobotHandle = obj.sim.getObject(robot_name);

            if(obj.RobotHandle==-1)
                disp('ERROR: Failed getting robot handle');
                error = 1;
                return;
            end

            %arm joints handles
            jointName{1} = '/waist';
            jointName{2} = '/shoulder';
            jointName{3} = '/elbow';
            jointName{4} = '/wrist_angle';
            [~,obj.number_of_joints]=size(jointName);
            
            for j=1:obj.number_of_joints
                obj.jointHandle{j}=obj.sim.getObject(jointName{j});
                if(obj.jointHandle{j}==-1)
                    disp('ERROR: Failed getting joint handle');
                    error = 1;
                    return
                end
            end

            fingerName{1} = '/left_finger';
            fingerName{2} = '/right_finger';
            %Get Fingers Handles
            for m=1:2
                obj.HandHandle{m} = obj.sim.getObject(fingerName{m});
                if(obj.HandHandle{m}==-1)
                    disp('ERROR: Failed getting finger handle');
                    error = 1;
                    return;
                end
            end

            if obj.ROS_WORKING==1
                %Initialialize comunication with ros
            end

        end

        %Function that allows you to get the arm characteristics
        function [nJoints,Link,MinPositionJoint,MaxPositionJoint] = get_RobotCharacteristics(obj)
            %Number of joints
            nJoints = obj.number_of_joints;
            %Min and max joint value for each joint
            MinPositionJoint(1)=-180*pi/180;
            MinPositionJoint(2)=-111*pi/180;
            MinPositionJoint(3)=-121*pi/180;
            MinPositionJoint(4)=-100*pi/180;
            MaxPositionJoint(1)=180*pi/180;
            MaxPositionJoint(2)=107*pi/180;
            MaxPositionJoint(3)=92*pi/180;
            MaxPositionJoint(4)=123*pi/180;     
            %Links lenghts
            Link(1)=0.08945;
            Link(2)=0.1;
            Link(3)=0.035;
            Link(4)=0.1;
            Link(5)=0.12915;
            obj.MinJointPos = MinPositionJoint;
            obj.MaxJointPos = MaxPositionJoint;
        end

        %Function that allows you to get the position of the arm (x,y,z)
        function armPosition=get_robot_position(obj)
            armPosition_str = obj.sim.getObjectPosition(obj.RobotHandle);
            armPosition(1) = armPosition_str{1};
            armPosition(2) = armPosition_str{2};
            armPosition(3) = armPosition_str{3};
        end

        %Function that allows you to set the joints from the arm
        function error = set_joints(obj, armJoints) %armJoints (1-4) in rad
            error = 0;
            for i=1:1:obj.number_of_joints
                if armJoints(i)<obj.MinJointPos(i) || armJoints(i)>obj.MaxJointPos(i)
                    disp('ERROR: value outside the limits of the joints');
                    error = 1;
                    return;
                else
                    obj.sim.setJointTargetPosition(obj.jointHandle{i},armJoints(i));
                end
            end

            if obj.ROS_WORKING==1
                %Send values for real robot
            end
        end

        %Function that allows you to get the joints values from the arm
        %armJoints - joint values for arm (in rad)
        function armJoints = get_joints(obj)
            for i = 1:obj.number_of_joints
                %Read from joints
                armJoints(i)=obj.sim.getJointPosition(obj.jointHandle{i});
            end

            if obj.ROS_WORKING==1
                %Send values for real robot
            end
        end

        %Functions that allows close the hand
        function close_hand(obj)       
            obj.sim.setJointTargetVelocity(obj.HandHandle{1},-0.05);
            obj.sim.setJointTargetVelocity(obj.HandHandle{2},0.05);

            if obj.ROS_WORKING==1
                %Send values to the gripper
            end
            
        end

        %Functions that allows open the hand
        function open_hand(obj)
            obj.sim.setJointTargetVelocity(obj.HandHandle{1},0.05);

            obj.sim.setJointTargetVelocity(obj.HandHandle{2},-0.05);

            if obj.ROS_WORKING==1
                %Send values to the gripper
            end
        end      
    end
end
