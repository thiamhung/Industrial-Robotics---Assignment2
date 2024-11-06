function collision = Collision_Dectection(robot,q_current,object)

% Set up link sizes
% Create a single figure and axis for both robot and ellipsoids

% ax = gca; % Create a single axes object
% hold on;
% axis equal;
% view(3);
% We can fix the coordinate to change the size of the elips around each link
secondRobotLinkSize = {[2.5/4, 0.5, 0.8/2], ...
    [2.5, 1.4, 1]/4, ...
    [1, 0.5, 0.25], ...
    [0.5, 0.5, 0.5], ...
    [0.25, 0.25, 1], ...
    [0.5,0.5,0.5], ...
    [0.25,0.25,0.25]};

  % restart the flag
            collision = 0;

            % Allocate variable
            linkTransform = zeros(4,4,robot.model.n+1);
            midpoints = zeros(4,4);
            numOfLinks = size(robot.model.links,2);

            % The joint position checked
            currentJoint = q_current;

            % Get the transform of each links
            linkTransform(:,:,1) = robot.model.base.T;
            L = robot.model.links;

% Loop through each joint configuration in qMatrix
     for i = 1:robot.model.n
    % Calculate transform for each link at configuration j
linkTransform(:,:,i+1) = linkTransform(:,:,i) * trotz(currentJoint(i) + L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
    end

    % Define ellipsoid midpoints for visualization
    for i = 1:numOfLinks+1
        % % To visualise the ellipsoid
                centerPoint = [0,0,0];
                [X,Y,Z] = ellipsoid(centerPoint(1),centerPoint(2),centerPoint(3),secondRobotLinkSize{i}(1),secondRobotLinkSize{i}(2),secondRobotLinkSize{i}(3));
                EllipsoidPoints = [X(:),Y(:),Z(:)];

%Correct the orientation of the ellipsoid

                if i == 1
                    midpoints(:,:,i) = linkTransform(:,:,i) * trotx(0);
                    base = robot.model.base.T;
                    midpoints(1:3,4,i) = (linkTransform(1:3,4,i) + base(1:3,4))/2;
                elseif i==2
                   midpoints(:,:,i) = linkTransform(:,:,i) * trotx(0);
                   % base = robot.model.base.T;
                   midpoints(1:3,4,i) = (linkTransform(1:3,4,i));
                   % midpoints(1:3,4,i) = (linkTransform(1:3,4,i) + base(1:3,4))/2;
                elseif i == 3
                    midpoints(:,:,i) = linkTransform(:,:,i) * trotx(0);
                    % base = robot.model.base.T;
                    midpoints(1:3,4,i) = (linkTransform(1:3,4,i)+linkTransform(1:3,4,i-1))/2;

                elseif i == 4
                    midpoints(:,:,i) = linkTransform(:,:,i) * trotx(0);
                    midpoints(1:3,4,i) = (linkTransform(1:3,4,i));

                elseif i == 5
                    midpoints(:,:,i) = linkTransform(:,:,i) * trotx(pi/2);
                    midpoints(1:3,4,i) = (linkTransform(1:3,4,i) + linkTransform(1:3,4,i-1))/2;
                elseif i==6
                    midpoints(:,:,i) = linkTransform(:,:,i) * trotx(pi/2);
                     midpoints(1:3,4,i) = (linkTransform(1:3,4,i));
                else
                    midpoints(:,:,i) = linkTransform(:,:,i);
                    midpoints(1:3,4,i) = (linkTransform(1:3,4,i) + linkTransform(1:3,4,i-1))/2;
                end

                % To visualise the ellipsoid
                EllipsoidPointsAndOnes = (midpoints(:,:,i) * [EllipsoidPoints,ones(size(EllipsoidPoints,1),1)]')';
                updatedEllipsoidPoints = EllipsoidPointsAndOnes(:,1:3);
                plot3(updatedEllipsoidPoints(:,1), updatedEllipsoidPoints(:,2), updatedEllipsoidPoints(:,3));
               
              
     end
   

          % Extract vertices from the object
    verts = get(object, 'Vertices');
    verts = [verts, ones(size(verts, 1), 1)];  % Add homogeneous coordinate

            for i = 1:numOfLinks+1

                invTr = inv(midpoints(:,:,i));
                    warning on
                for k = i:size(verts)
                    warning off
                    vertTr = midpoints(:,:,i);
                    vertTr(1:3,4) = [verts(k,1:3)]';  % Keep rotation from links but change the position to the vertex of ply file
                    linksToVerts = invTr * vertTr;

                    % Check any collision - if all true then collision detected
                    if (abs(linksToVerts(1,4)) <= abs(secondRobotLinkSize{i}(1))) && ...
                       (abs(linksToVerts(2,4)) <= abs(secondRobotLinkSize{i}(2))) && ...
                       (abs(linksToVerts(3,4)) <= abs(secondRobotLinkSize{i}(3)))

                    %     % return the collision flag
                    %     collision = 1;
                    %     if collision == 1
                    %         app.collisionFlag = true;
                    %     end
                    % 
                    %     % checking flags
                    %     % disp(collision)
                    %     % disp(app.collisionFlag)
                    % 
                    %     return
                    % end
                    disp("intersection detected");
                    collision = 1;
                    disp(["Intersection link number: ", i]);
                    % disp(k);
                    disp("Collision detected at: ");
                    disp(["x:", verts(k,1), "y: ", verts(k,2), "z: ", verts(k,3)]);
                    break
                    end
                end

   
            end
end