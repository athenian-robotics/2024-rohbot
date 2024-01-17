package frc.robot.lib;

import io.github.jdiemke.triangulation.DelaunayTriangulator;
import io.github.jdiemke.triangulation.NotEnoughPointsException;
import io.github.jdiemke.triangulation.Triangle2D;
import io.github.jdiemke.triangulation.Vector2D;
import java.util.List;

public class BarycentricInterpolation {
  private final DelaunayTriangulator triangulator;
  private final double[] z;

  public BarycentricInterpolation(double[] x, double[] y, double[] z)
      throws NotEnoughPointsException {
    this.z = z;

    List<Vector2D> vertices = new java.util.ArrayList<>(List.of());
    for (int i = 0; i < x.length; i++) {
      vertices.add(new Vector2D(x[i], y[i]));
    }
    triangulator = new DelaunayTriangulator(vertices);
    triangulator.triangulate();
  }

  public double interpolate(double xVal, double yVal) {
    System.out.println(triangulator.getTriangles());
    var triangles = triangulator.getTriangles();
    Triangle2D containingTriangle =
        triangles.stream()
            .filter(triangle -> contains(triangle, new Vector2D(xVal, yVal)))
            .findFirst()
            .orElseThrow();

    Vector2D p1 = containingTriangle.a;
    Vector2D p2 = containingTriangle.b;
    Vector2D p3 = containingTriangle.c;

    // Calculate barycentric coordinates
    double detT = (p2.y - p3.y) * (p1.x - p3.x) + (p3.x - p2.x) * (p1.y - p3.y);
    double alpha = ((p2.y - p3.y) * (xVal - p3.x) + (p3.x - p2.x) * (yVal - p3.y)) / detT;
    double beta = ((p3.y - p1.y) * (xVal - p3.x) + (p1.x - p3.x) * (yVal - p3.y)) / detT;
    double gamma = 1 - alpha - beta;

    // Interpolate z value
    return alpha * z[triangles.indexOf(containingTriangle) * 3]
        + beta * z[triangles.indexOf(containingTriangle) * 3 + 1]
        + gamma * z[triangles.indexOf(containingTriangle) * 3 + 2];
  }

  private boolean contains(Triangle2D triangle, Vector2D vector2D) {
    return triangle.contains(vector2D)
        || isOn(triangle.a, triangle.b, vector2D)
        || isOn(triangle.b, triangle.c, vector2D)
        || isOn(triangle.c, triangle.a, vector2D);
  }

  private boolean isOn(Vector2D b, Vector2D c, Vector2D vector2D) {
    // Calculate vectors from b to c and b to vector2D
    Vector2D bc = c.sub(b);
    Vector2D bv = vector2D.sub(b);

    // Check if vector2D is on the line segment
    double crossProduct = bc.cross(bv);
    if (Math.abs(crossProduct) > 1e-9) {
      return false; // Not collinear
    }

    // Check if vector2D is within the bounding box of the line segment
    double dotProduct = bc.dot(bv);
    return !(dotProduct < 0) && !(dotProduct > bc.mag() * bc.mag()); // Outside the line segment
  }

  public static void main(String[] args) throws NotEnoughPointsException {
    // Example usage
    double[] xValues = {0, 0, 1};
    double[] yValues = {0, 1, 0};
    double[] zValues = {10, 20, 30};

    BarycentricInterpolation interpolator = new BarycentricInterpolation(xValues, yValues, zValues);

    double result = interpolator.interpolate(0, 0);
    System.out.println("Interpolated value: " + result);
  }
}
