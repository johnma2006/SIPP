import physics.Vect;


public class Node {

    public Vect position;
    public Double[] safeInterval;
    public double time;
    public Node parent;
    
    public Node(Vect position, Double[] safeInterval, double time) {
        this.position = position;
        this.safeInterval = safeInterval;
        this.time = time;
    }
    
    @Override
    public boolean equals(Object other) {
        double EPS = 0.001;
        if (!(other instanceof Node))
            return false;
        Node otherNode = (Node)other;
        if (Math.abs(otherNode.position.x()-this.position.x()) > EPS
                || Math.abs(otherNode.position.y()-this.position.y()) > EPS
                || Math.abs(otherNode.safeInterval[0]-this.safeInterval[0]) > EPS
                || Math.abs(otherNode.safeInterval[1]-this.safeInterval[1]) > EPS)
            return false;
        return true;
    }
    
    @Override
    public int hashCode() {
        return (int)(1543 * position.x()) + (int)(3079 * position.y());
    }
    
    @Override
    public String toString() {
        return position.toString() + ", [" + safeInterval[0] + "," + safeInterval[1] + "], time=" + time;
//        return position.toString() + ", time=" + time;
    }
}
