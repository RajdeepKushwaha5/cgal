// Regression test for https://github.com/CGAL/cgal/issues/9311
//
// write_OBJ() succeeds on empty input (returns true).
// read_OBJ() was not symmetric — it returned false for an empty-but-valid file.
// This test verifies the fix: reading an empty OBJ file must also return true.

#include <CGAL/Simple_cartesian.h>
#include <CGAL/IO/OBJ.h>

#include <cassert>
#include <sstream>
#include <vector>

using K     = CGAL::Simple_cartesian<double>;
using Point = K::Point_3;
using Face  = std::vector<std::size_t>;

int main()
{
  std::vector<Point> points;
  std::vector<Face>  polygons;

  // Round-trip: write empty geometry, then read it back — both must succeed.
  std::ostringstream oss;
  bool ok = CGAL::IO::write_OBJ(oss, points, polygons);
  assert(ok && "write_OBJ failed on empty input");

  std::istringstream iss(oss.str());
  ok = CGAL::IO::read_OBJ(iss, points, polygons);
  assert(ok && "read_OBJ failed on the empty file produced by write_OBJ (issue #9311)");
  assert(points.empty()   && "expected empty points after reading empty OBJ");
  assert(polygons.empty() && "expected empty polygons after reading empty OBJ");

  // A file with only comments is also semantically empty — must succeed.
  std::istringstream iss2("# comment only\n# another comment\n");
  ok = CGAL::IO::read_OBJ(iss2, points, polygons);
  assert(ok && "read_OBJ failed on a comments-only OBJ file");

  std::cout << "Test for issue #9311 passed." << std::endl;
  return EXIT_SUCCESS;
}
