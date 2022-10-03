classdef LegIK < handle
    properties
        legtrees
        iks
        initial_guess
    end
    methods
        function IK = LegIK()
            buildIKSolver(IK);
        end
        function buildIKSolver(IK)
            % build rigidBody tree for each leg using Mini Cheetah urdf
            % model
            robot = importrobot("mini_cheetah_mesh.urdf");
            robot.DataFormat = 'column';
            legTrees{1} = subtree(robot, 'abduct_fr');
            legTrees{2} = subtree(robot, 'abduct_fl');            
            legTrees{3} = subtree(robot, 'abduct_hr');
            legTrees{4} = subtree(robot, 'abduct_hl');   
            IK.legtrees = legTrees;            
            % need to add a virtual toe with fixed joint in order to
            % compute inverse kinematics at foot
            params = getMiniCheetahParams();
            shankLength = params.kneeLinkLength;
            toe = rigidBody('toe');
            toeJnt = rigidBodyJoint('toe','fixed');
            setFixedTransform(toeJnt, trvec2tform([0, 0, -shankLength]));
            toe.Joint = toeJnt;
            for leg = 1:4
                addBody(legTrees{leg}, toe, legTrees{leg}.BodyNames{end});
            end
            % build inverse kinematics solver for each leg
            IK.iks{1} = inverseKinematics('RigidBodyTree',legTrees{1});
            IK.iks{2} = inverseKinematics('RigidBodyTree',legTrees{2});
            IK.iks{3} = inverseKinematics('RigidBodyTree',legTrees{3});
            IK.iks{4} = inverseKinematics('RigidBodyTree',legTrees{4});
            IK.initial_guess = [0, -0.8, 1.7]'; % initial guess to bias the iks
        end   
        function q = solve(IK, p, leg)
            % solve inverse kinematics for leg
            [q, info] = IK.iks{leg}(IK.legtrees{leg}.BodyNames{end}, trvec2tform(p), [0,0,0,1,1,1]',IK.initial_guess);
            if ~strcmp(info.Status, 'success')
                fprintf("Failed to solve the ik \n");
                fprintf("Use the default joint angle \n");
                q = IK.initial_guess;
            end
        end
    end
end