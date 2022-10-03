classdef SingleRigidBodyDyn < handle
    properties
        physic_params
        treemodel
        I % Trunk inertia in body frame
        m % Trunk mass
        gvec % gravity vector       
    end

    methods
        function H = SingleRigidBodyDyn()
            physic_params = getMiniCheetahParams();
            H.m = physic_params.bodyMass;
            H.I = physic_params.bodyRotInertia;
            H.gvec = [0,0,-9.81]';
            H.physic_params = physic_params;
            H.treemodel = buildTreeModel(physic_params);
        end

        function [xnext,y] = dynamics(H, x,u,dt,ctact,pf)
            xnext = SRBdyn(x, u, dt, ctact, pf);
            y = 0;          
        end

        function [A,B,C,D] = dynamics_par(H, x, u, dt, ctact, pf)
            [A,B] = SRBdyn_par(x, u, dt, ctact, pf);
            C = 0;
            D = 0;
        end

%         function prel = foothold_loc_rel_CoM(H, x, leg)
%             % Get foothold location relative to CoM in body frame
%             % This function assumes leg is in stance
%             eul = x(1:3);
%             pCoM = x(4:6);
%             qdummy = x(13:24);
%             Rbody = eul2Rot(eul);
%             pf = qdummy(3*(leg-1)+1:3*leg);
%             prel_global = pf - pCoM;           
%             prel = Rbody'*prel_global;
%         end                 
    end
end