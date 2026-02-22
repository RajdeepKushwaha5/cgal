// Copyright (c) 2026  GeometryFactory (France).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org)
//
// $URL$
// $Id$
// SPDX-License-Identifier: LGPL-3.0-or-later OR LicenseRef-Commercial
//
// Author(s)     : Rajdeep Singh

// This test requires tinygltf on the include path and CGAL_LINKED_WITH_GLTF
// defined.  Without these the test is a no-op so it does not break ordinary
// CI builds that do not have tinygltf available.

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/IO/GLTF.h>

#include <cassert>
#include <cstdio>   // std::tmpnam / std::remove
#include <iostream>
#include <sstream>
#include <vector>

#ifdef CGAL_LINKED_WITH_GLTF

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3                                          Point_3;
typedef std::vector<std::size_t>                            Face;

// Build a simple unit tetrahedron polygon soup.
void make_tetrahedron(std::vector<Point_3>& pts, std::vector<Face>& faces)
{
  pts  = { Point_3(0,0,0), Point_3(1,0,0), Point_3(0,1,0), Point_3(0,0,1) };
  faces = { {0,1,2}, {0,1,3}, {0,2,3}, {1,2,3} };
}

// --------------------------------------------------------------------------
// Test 1: round-trip through ASCII GLTF stream
// --------------------------------------------------------------------------
void test_roundtrip_ascii()
{
  std::cout << "test_roundtrip_ascii ... ";

  std::vector<Point_3> pts_in, pts_out;
  std::vector<Face>    faces_in, faces_out;
  make_tetrahedron(pts_in, faces_in);

  // Write to stringstream.
  std::ostringstream oss;
  bool ok = CGAL::IO::write_GLTF(oss, pts_in, faces_in, CGAL::parameters::verbose(true));
  assert(ok);
  assert(!oss.str().empty());

  // Read back.
  std::istringstream iss(oss.str());
  ok = CGAL::IO::read_GLTF(iss, pts_out, faces_out, CGAL::parameters::verbose(true));
  assert(ok);
  assert(pts_out.size()   == pts_in.size());
  assert(faces_out.size() == faces_in.size());

  // Spot-check coordinates (float round-trip has ~1e-7 relative error).
  for(std::size_t i = 0; i < pts_in.size(); ++i)
  {
    assert(std::abs(CGAL::to_double(pts_out[i].x()) - CGAL::to_double(pts_in[i].x())) < 1e-5);
    assert(std::abs(CGAL::to_double(pts_out[i].y()) - CGAL::to_double(pts_in[i].y())) < 1e-5);
    assert(std::abs(CGAL::to_double(pts_out[i].z()) - CGAL::to_double(pts_in[i].z())) < 1e-5);
  }

  std::cout << "OK\n";
}

// --------------------------------------------------------------------------
// Test 2: empty polygon soup — write succeeds, read returns empty ranges
// --------------------------------------------------------------------------
void test_empty_roundtrip()
{
  std::cout << "test_empty_roundtrip ... ";

  std::vector<Point_3> pts_in, pts_out;
  std::vector<Face>    faces_in, faces_out;

  std::ostringstream oss;
  bool ok = CGAL::IO::write_GLTF(oss, pts_in, faces_in);
  assert(ok);

  std::istringstream iss(oss.str());
  ok = CGAL::IO::read_GLTF(iss, pts_out, faces_out);
  assert(ok);
  assert(pts_out.empty());
  assert(faces_out.empty());

  std::cout << "OK\n";
}

// --------------------------------------------------------------------------
// Test 3: non-existent file should return false
// --------------------------------------------------------------------------
void test_nonexistent_file()
{
  std::cout << "test_nonexistent_file ... ";

  std::vector<Point_3> pts;
  std::vector<Face>    faces;
  bool ok = CGAL::IO::read_GLTF("this_file_does_not_exist_cgal.gltf", pts, faces);
  assert(!ok);

  std::cout << "OK\n";
}

// --------------------------------------------------------------------------
// Test 4: write to a file, read it back (exercises the filename overloads)
// --------------------------------------------------------------------------
void test_file_overloads()
{
  std::cout << "test_file_overloads ... ";

  std::vector<Point_3> pts_in, pts_out;
  std::vector<Face>    faces_in, faces_out;
  make_tetrahedron(pts_in, faces_in);

  const std::string fname = "cgal_test_gltf_tmp.gltf";

  bool ok = CGAL::IO::write_GLTF(fname, pts_in, faces_in);
  assert(ok);

  ok = CGAL::IO::read_GLTF(fname, pts_out, faces_out);
  assert(ok);
  assert(pts_out.size()   == pts_in.size());
  assert(faces_out.size() == faces_in.size());

  std::remove(fname.c_str());

  std::cout << "OK\n";
}

// --------------------------------------------------------------------------
// Test 5: data is actually appended (istream overload does not clear soup)
// --------------------------------------------------------------------------
void test_append_semantics()
{
  std::cout << "test_append_semantics ... ";

  std::vector<Point_3> pts_in, accumulated_pts;
  std::vector<Face>    faces_in, accumulated_faces;
  make_tetrahedron(pts_in, faces_in);

  // Write once.
  std::ostringstream oss;
  bool ok = CGAL::IO::write_GLTF(oss, pts_in, faces_in);
  assert(ok);

  // Read the same data twice into the same containers.
  for(int i = 0; i < 2; ++i)
  {
    std::istringstream iss(oss.str());
    ok = CGAL::IO::read_GLTF(iss, accumulated_pts, accumulated_faces);
    assert(ok);
  }

  assert(accumulated_pts.size()   == 2 * pts_in.size());
  assert(accumulated_faces.size() == 2 * faces_in.size());

  std::cout << "OK\n";
}

#endif // CGAL_LINKED_WITH_GLTF

int main()
{
#ifdef CGAL_LINKED_WITH_GLTF
  test_roundtrip_ascii();
  test_empty_roundtrip();
  test_nonexistent_file();
  test_file_overloads();
  test_append_semantics();

  std::cout << "All GLTF tests passed.\n";
#else
  std::cout << "GLTF support not enabled (CGAL_LINKED_WITH_GLTF not defined); "
               "skipping tests.\n";
#endif
  return EXIT_SUCCESS;
}
