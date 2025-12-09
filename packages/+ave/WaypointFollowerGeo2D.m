classdef WaypointFollowerGeo2D < handle
    methods
        function this = WaypointFollowerGeo2D(maxNumWaypoints)
            %WaypointFollowerGeo2D Construct a new waypoint follower for a geographic WGS84 coordinate system.
            % 
            % PARAMETER
            % maxNumWaypoints ... The maximum number of waypoints that can be stored.
            % 
            % NOTE
            % Run the LoadWaypoints member function to load or reload waypoints to this waypoint follower. Afterwards, run the
            % Step member function periodically to calculate the current look-ahead point.
            arguments
                maxNumWaypoints (1,1) uint32
            end
            this.waypoints = zeros(2, maxNumWaypoints);
            this.num_waypoints = int32(0);
            this.idx_active_line = int32(0);
        end
        function LoadWaypoints(this, waypoints)
            %LoadWaypoints Load or reload all waypoints until the first non-finite point. If the given waypoints to load are larger
            % than the internal waypoint storage, elements are discarded. Only unique points are copied to the internal storage.
            % 
            % PARAMETER
            % waypoints ... 2-by-N matrix of N waypoints to be loaded. The first non-finite waypoint indicates the end of waypoints.
            arguments
                this ave.WaypointFollowerGeo2D
                waypoints (2,:) double
            end
            this.num_waypoints = int32(0);
            max_num_waypoints = int32(size(this.waypoints,2));
            N = int32(size(waypoints,2));
            for i = int32(1):N
                if(~isfinite(waypoints(1,i)) || ~isfinite(waypoints(2,i)))
                    break;
                end
                if(this.num_waypoints)
                    d = waypoints(:, i) - this.waypoints(:, this.num_waypoints);
                    if(norm(d) <= eps)
                        continue;
                    end
                end
                this.num_waypoints = this.num_waypoints + int32(1);
                this.waypoints(:, this.num_waypoints) = waypoints(:, i);
                if(this.num_waypoints >= max_num_waypoints)
                    break;
                end
            end
            this.idx_active_line = min(int32(1), this.num_waypoints - int32(1));
        end
        function [lookAheadPoint, bearingToLookAheadPoint, lineAngleAtLookAheadPoint, idxLine] = Step(this, position, lookAheadDistance)
            %Step Calculate the current look ahead point based on the current position and update the active line segment. Call this function each
            % time step to update the waypoint follower.
            % 
            % PARAMETER
            % position ... Current geographic position given as [latitude; longitude; altitude].
            % lookAheadDistance ... Look ahead distance, e.g. radius of the circle around the current position.
            % 
            % RETURN
            % lookAheadPoint ... Geographic position (lat, lon), that is at most lookAheadDistance away from the current position.
            % bearingToLookAheadPoint ... Angle from the current position to the look ahead point.
            % lineAngleAtLookAheadPoint ... Angle of the line segment that intersects with the circle.
            % idxLine ... Index of the line that intersects with the circle or zero if internal waypoints contains less than two points.
            % 
            % DETAILS
            % The waypoint follower calculates the intersection point (also known as the look ahead point) of the active line in the waypoint list with the circle around the current position.
            % At the start, the active line corresponds to the first line in the waypoint list.
            % Once the end of the active line is reached, the following line segment is set as the new active line.
            % If there is no intersection of the circle with the active line, the closest point to the active line is selected.
            % The distance from the current position to the look ahead point is at most the look ahead radius.
            % If the last line segment is reached and the last waypoint is within the circle, then that last waypoint corresponds to the look ahead point.
            % A line segment that has been processed is never selected again as an active line segment.
            arguments
                this ave.WaypointFollowerGeo2D
                position (3,1) double
                lookAheadDistance (1,1) double
            end

            % default output
            lookAheadPoint = position;
            bearingToLookAheadPoint = 0.0;
            lineAngleAtLookAheadPoint = 0.0;
            idxLine = int32(0);

            % special treatment if not enough waypoints
            if(int32(0) == this.num_waypoints)
                return;
            end
            if(int32(1) == this.num_waypoints)
                lookAheadPoint = this.ClampDistance(position(1:2), this.waypoints(:,1), lookAheadDistance, position(3));
                bearingToLookAheadPoint = this.Bearing(position(1:2), lookAheadPoint, position(3));
                lineAngleAtLookAheadPoint = bearingToLookAheadPoint;
                return;
            end

            % current line segment
            w1 = this.waypoints(:, this.idx_active_line);
            w2 = this.waypoints(:, this.idx_active_line + int32(1));
            [s, ps] = this.LineCircleIntersectWGS84(w1, w2, position, lookAheadDistance);

            % special treatment if circle does not intersect with the current line
            if(~isfinite(s(1)) || ~isfinite(s(2)) || ((s(1) < 0) && (s(2) < 0)) || ((s(1) > 1) && (s(2) > 1)))
                closestPoint = this.ClosestPointToLineWGS84(position, w1, w2);
                lookAheadPoint = this.ClampDistance(position(1:2), closestPoint, lookAheadDistance, position(3));
                bearingToLookAheadPoint = this.Bearing(position(1:2), lookAheadPoint, position(3));
                lineAngleAtLookAheadPoint = this.Bearing(w1, w2, position(3));
                idxLine = this.idx_active_line;
                return;
            end

            % find the next line segment that intersects with the circle
            while(true)
                if((s(2) >= 0) && (s(2) < 1))
                    lookAheadPoint = ps;
                    break;
                end
                if(this.idx_active_line >= (this.num_waypoints - int32(1)))
                    lookAheadPoint = w2;
                    break;
                end
                this.idx_active_line = this.idx_active_line + int32(1);
                w1 = this.waypoints(:, this.idx_active_line);
                w2 = this.waypoints(:, this.idx_active_line + int32(1));
                [s, ps] = this.LineCircleIntersectWGS84(w1, w2, position, lookAheadDistance);
            end
            bearingToLookAheadPoint = this.Bearing(position(1:2), lookAheadPoint, position(3));
            lineAngleAtLookAheadPoint = this.Bearing(w1, w2, position(3));
            idxLine = this.idx_active_line;
        end
        function [waypoints, numWaypoints] = GetWaypoints(this)
            %GetWaypoints Get the loaded waypoints.
            % 
            % RETURN
            % waypoints ... 2-by-N matrix of N waypoints, where N denotes the maximum number of waypoints that can be stored to this class.
            %               Unused waypoints are replaced by NaN.
            % numWaypoints ... The number of finite waypoints in the waypoints output.
            arguments
                this ave.WaypointFollowerGeo2D
            end
            waypoints = this.waypoints;
            numWaypoints = this.num_waypoints;
            waypoints(:,(numWaypoints+1):end) = NaN;
        end
        function plot(this, varargin)
            %plot Plot the loaded waypoints.
            assert(isa(this, 'ave.WaypointFollowerGeo2D'));
            plot(this.waypoints(1, 1:this.num_waypoints), this.waypoints(2, 1:this.num_waypoints), varargin{:});
        end
    end
    methods(Access = protected)
        function p = ClampDistance(~, p1, p2, radius, altitude)
            p = p2;
            [N, E, ~] = ave.LLA2NED(p2(1), p2(2), altitude, p1(1), p1(2), altitude);
            d = [N; E];
            L = norm(d);
            if(L > radius)
                d = radius/L * d;
                [lat, lon, ~] = ave.NED2LLA(d(1), d(2), 0, p1(1), p1(2), altitude);
                p = [lat; lon];
            end
        end
        function b = Bearing(~, pointA, pointB, altitude)
            [N, E, ~] = ave.LLA2NED(pointB(1), pointB(2), altitude, pointA(1), pointA(2), altitude);
            b = atan2(E, N);
        end
        function [s, ps] = LineCircleIntersectWGS84(~, p1, p2, position, R)
            [N1, E1, ~] = ave.LLA2NED(p1(1), p1(2), position(3), position(1), position(2), position(3));
            [N2, E2, ~] = ave.LLA2NED(p2(1), p2(2), position(3), position(1), position(2), position(3));
            w1 = [N1; E1];
            w2 = [N2; E2];
            s = ave.geometry.LineCircleIntersect(w1, w2, [0; 0], R);
            v = w1 + s(2) * (w2 - w1);
            [lat, lon, ~] = ave.NED2LLA(v(1), v(2), 0, position(1), position(2), position(3));
            ps = [lat; lon];
        end
        function closestPoint = ClosestPointToLineWGS84(~, position, p1, p2)
            [N1, E1, ~] = ave.LLA2NED(p1(1), p1(2), position(3), position(1), position(2), position(3));
            [N2, E2, ~] = ave.LLA2NED(p2(1), p2(2), position(3), position(1), position(2), position(3));
            w1 = [N1; E1];
            w2 = [N2; E2];
            [p, ~] = ave.geometry.ClosestPointToLine([0; 0], w1, w2);
            [lat, lon, ~] = ave.NED2LLA(p(1), p(2), 0, position(1), position(2), position(3));
            closestPoint = [lat; lon];
        end
    end
    properties(Access = protected)
        waypoints (2,:) double
        num_waypoints (1,1) int32
        idx_active_line (1,1) int32
    end
end
