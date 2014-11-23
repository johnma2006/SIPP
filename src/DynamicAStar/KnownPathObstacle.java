package DynamicAStar;
import java.util.ArrayList;
import java.util.List;

import physics.Vect;


public class KnownPathObstacle implements Obstacle {

    public double radius;
    public List<Node> path;

    public KnownPathObstacle(List<Node> path, double radius) {
        this.radius = radius;
        this.path = path;
    }

    @Override
    public List<Double[]> getCollisionIntervals(Vect point, double robotRadius) {
        robotRadius += AVOID_RADIUS;
        List<Double[]> collisionIntervals = new ArrayList<Double[]>();

        for (int i = 0; i < path.size() - 1; i++) {
            Vect position = path.get(i).position;
            Vect velocity = path.get(i+1).position.minus(path.get(i).position).times(1.0 / (path.get(i+1).time - path.get(i).time));
            Double[] collisionInterval = Obstacle.getTimeIntersectCircleAroundOrigin(position.minus(point), velocity, robotRadius+radius);
            if (collisionInterval != null) {
                double t1 = Math.max(collisionInterval[0] + path.get(i).time, path.get(i).time);
                double t2 = Math.min(collisionInterval[1] + path.get(i).time, path.get(i+1).time);
                if (t2 > t1) {
                    if (collisionIntervals.size() > 0)
                        if (collisionIntervals.get(collisionIntervals.size() - 1)[1] == t1)
                            t1 = collisionIntervals.remove(collisionIntervals.size() - 1)[0];
                    collisionIntervals.add(new Double[]{t1, t2});
                }
            }
        }
        return collisionIntervals;
    }
    
    @Override
    public boolean checkIfCollidesWithRobot(Vect robotOrigin, Vect robotVelocity, double robotRadius, double startMovingTime, double endMovingTime) {
        robotRadius += AVOID_RADIUS;
        for (int i = 0; i < path.size() - 1; i++) {
            Vect position = path.get(i).position;
            Vect velocity = path.get(i+1).position.minus(path.get(i).position).times(1.0 / (path.get(i+1).time - path.get(i).time));
            Vect relativePoint = position.minus(robotOrigin);
            Vect relativeVelocity = velocity.minus(robotVelocity);
            Double[] collisionInterval = Obstacle.getTimeIntersectCircleAroundOrigin(relativePoint, relativeVelocity, robotRadius+radius);
            if (collisionInterval != null) {
                double t1 = Math.max(collisionInterval[0] + path.get(i).time, path.get(i).time);
                double t2 = Math.min(collisionInterval[1] + path.get(i).time, path.get(i+1).time);
                if (t2 > t1 && t1 <= endMovingTime && t2 >= startMovingTime)
                    return true;
            }
        }
        return false;
    }

    @Override
    public void updatePosition(double time) {

    }

    @Override
    public double getRadius() {
        return this.radius;
    }

    @Override
    public Vect getPosition() {
        return null;
    }

    @Override
    public Vect getVelocity() {
        return null;
    }

    @Override
    public void setPosition(Vect newPosition) {

    }

    @Override
    public void setVelocity(Vect newVelocity) {

    }
}
