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

#ifndef CGAL_IO_GLTF_H
#define CGAL_IO_GLTF_H

#include <CGAL/IO/helpers.h>
#include <CGAL/Named_function_parameters.h>
#include <CGAL/boost/graph/named_params_helper.h>
#include <CGAL/Container_helper.h>

#include <boost/range/value_type.hpp>

#include <algorithm>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <type_traits>
#include <vector>

#ifdef CGAL_LINKED_WITH_GLTF
#include <tiny_gltf.h>
#endif

namespace CGAL {
namespace IO {

#if defined(CGAL_LINKED_WITH_GLTF) || defined(DOXYGEN_RUNNING)

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
// Read

namespace internal {

// Returns true iff the first four bytes of `data` are the GLB magic number "glTF" (0x46546C67).
inline bool is_glb_magic(const std::vector<uint8_t>& data)
{
  static const uint8_t magic[4] = {0x67, 0x6C, 0x54, 0x46}; // "glTF"
  return data.size() >= 4 &&
         data[0] == magic[0] && data[1] == magic[1] &&
         data[2] == magic[2] && data[3] == magic[3];
}

// Extract typed scalar elements from a tinygltf accessor into an output vector.
// Handles component types BYTE, UNSIGNED_BYTE, SHORT, UNSIGNED_SHORT,
// UNSIGNED_INT, and FLOAT.
template <typename T>
bool fill_from_accessor(const tinygltf::Model& model,
                        int accessor_idx,
                        std::vector<T>& out)
{
  if(accessor_idx < 0) return false;

  const tinygltf::Accessor&   acc  = model.accessors[accessor_idx];
  const tinygltf::BufferView& bv   = model.bufferViews[acc.bufferView];
  const tinygltf::Buffer&     buf  = model.buffers[bv.buffer];

  const int component_type = acc.componentType;
  const std::size_t stride = bv.byteStride != 0
                             ? bv.byteStride
                             : tinygltf::GetComponentSizeInBytes(component_type);

  out.resize(acc.count);
  const uint8_t* raw = buf.data.data() + bv.byteOffset + acc.byteOffset;

  for(std::size_t i = 0; i < acc.count; ++i, raw += stride)
  {
    switch(component_type)
    {
    case TINYGLTF_COMPONENT_TYPE_BYTE:
      out[i] = static_cast<T>(*reinterpret_cast<const int8_t*>(raw)); break;
    case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE:
      out[i] = static_cast<T>(*reinterpret_cast<const uint8_t*>(raw)); break;
    case TINYGLTF_COMPONENT_TYPE_SHORT:
      out[i] = static_cast<T>(*reinterpret_cast<const int16_t*>(raw)); break;
    case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT:
      out[i] = static_cast<T>(*reinterpret_cast<const uint16_t*>(raw)); break;
    case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT:
      out[i] = static_cast<T>(*reinterpret_cast<const uint32_t*>(raw)); break;
    case TINYGLTF_COMPONENT_TYPE_FLOAT:
      out[i] = static_cast<T>(*reinterpret_cast<const float*>(raw)); break;
    default:
      return false;
    }
  }
  return true;
}

// Core read function.  Accepts bytes (ASCII GLTF or binary GLB) as a
// contiguous buffer and populates a polygon soup.
//
// \pre `base_path` is used by tinygltf to resolve external buffer URIs.
//       Pass an empty string when loading self-contained files (embedded
//       base64 buffers or GLB).
template <typename PointRange, typename PolygonRange>
bool read_GLTF_impl(const std::vector<uint8_t>& raw,
                    PointRange&                  points,
                    PolygonRange&                polygons,
                    const std::string&           base_path,
                    const bool                   verbose)
{
  tinygltf::TinyGLTF loader;
  tinygltf::Model    model;
  std::string        err, warn;

  bool ok = false;
  if(is_glb_magic(raw))
  {
    ok = loader.LoadBinaryFromMemory(&model, &err, &warn,
                                     raw.data(), static_cast<uint32_t>(raw.size()),
                                     base_path);
  }
  else
  {
    const std::string text(raw.begin(), raw.end());
    ok = loader.LoadASCIIFromString(&model, &err, &warn,
                                    text.c_str(), static_cast<uint32_t>(text.size()),
                                    base_path);
  }

  if(!warn.empty() && verbose)
    std::cerr << "GLTF warning: " << warn << std::endl;

  if(!ok)
  {
    if(verbose)
      std::cerr << "GLTF error: " << err << std::endl;
    return false;
  }

  // Read each mesh primitive.  Only TRIANGLES mode (4) is supported as
  // polygon soup.  Other primitive types (lines, points, strips) are skipped.
  using Point_3   = typename boost::range_value<PointRange>::type;
  using Polygon_t = typename boost::range_value<PolygonRange>::type;
  using Index_t   = typename boost::range_value<Polygon_t>::type;

  for(const tinygltf::Mesh& mesh : model.meshes)
  {
    for(const tinygltf::Primitive& prim : mesh.primitives)
    {
      if(prim.mode != TINYGLTF_MODE_TRIANGLES)
      {
        if(verbose)
          std::cerr << "GLTF: skipping non-triangle primitive (mode="
                    << prim.mode << ")." << std::endl;
        continue;
      }

      auto pos_it = prim.attributes.find("POSITION");
      if(pos_it == prim.attributes.end())
      {
        if(verbose)
          std::cerr << "GLTF: primitive has no POSITION attribute, skipping." << std::endl;
        continue;
      }

      // --- positions ---
      const tinygltf::Accessor&   pos_acc = model.accessors[pos_it->second];
      const tinygltf::BufferView& pos_bv  = model.bufferViews[pos_acc.bufferView];
      const tinygltf::Buffer&     pos_buf = model.buffers[pos_bv.buffer];

      const std::size_t vertex_base = points.size(); // offset for index remapping
      const uint8_t* pos_raw = pos_buf.data.data() + pos_bv.byteOffset + pos_acc.byteOffset;
      const std::size_t pos_stride = pos_bv.byteStride != 0 ? pos_bv.byteStride : 3 * sizeof(float);

      for(std::size_t i = 0; i < pos_acc.count; ++i)
      {
        const float* xyz = reinterpret_cast<const float*>(pos_raw + i * pos_stride);
        points.push_back(Point_3(xyz[0], xyz[1], xyz[2]));
      }

      // --- indices ---
      if(prim.indices < 0)
      {
        // No index buffer: implicit 0,1,2 / 3,4,5 / ... triangles.
        for(std::size_t i = 0; i + 2 < pos_acc.count; i += 3)
        {
          Polygon_t tri;
          CGAL::internal::resize(tri, 3);
          tri[0] = static_cast<Index_t>(vertex_base + i);
          tri[1] = static_cast<Index_t>(vertex_base + i + 1);
          tri[2] = static_cast<Index_t>(vertex_base + i + 2);
          polygons.push_back(tri);
        }
      }
      else
      {
        std::vector<std::size_t> indices;
        if(!fill_from_accessor(model, prim.indices, indices))
        {
          if(verbose)
            std::cerr << "GLTF: failed to read index buffer, skipping primitive." << std::endl;
          continue;
        }

        for(std::size_t i = 0; i + 2 < indices.size(); i += 3)
        {
          Polygon_t tri;
          CGAL::internal::resize(tri, 3);
          tri[0] = static_cast<Index_t>(vertex_base + indices[i]);
          tri[1] = static_cast<Index_t>(vertex_base + indices[i + 1]);
          tri[2] = static_cast<Index_t>(vertex_base + indices[i + 2]);
          polygons.push_back(tri);
        }
      }
    }
  }

  return true;
}

} // namespace internal

// ------------------------------------------------------------------------------------------------
// Public read API
// ------------------------------------------------------------------------------------------------

/*!
 * \ingroup PkgStreamSupportIoFuncsGLTF
 *
 * \brief reads the content of `is` into `points` and `polygons`, using the
 *        \ref IOStreamGLTF.
 *
 * Both ASCII GLTF (`.gltf`) and binary GLB (`.glb`) streams are accepted.
 * Only triangle primitives (`GL_TRIANGLES` mode) are read; all other
 * primitive types are silently skipped.
 *
 * \attention The polygon soup is not cleared; data from the stream are appended.
 * \attention To read a binary GLB stream, the flag `std::ios::binary` must be
 *            set when creating the `ifstream`.
 *
 * \tparam PointRange a model of the concepts `RandomAccessContainer` and
 *         `BackInsertionSequence` whose value type is the point type.
 * \tparam PolygonRange a model of the concept `SequenceContainer` whose
 *         `value_type` is itself a model of `SequenceContainer` whose
 *         `value_type` is an unsigned integer type convertible to `std::size_t`.
 * \tparam NamedParameters a sequence of \ref bgl_namedparameters "Named Parameters"
 *
 * \param is the input stream
 * \param points points of the polygon soup
 * \param polygons a range of triangles; each triangle uses the indices of the
 *        points in `points`.
 * \param np optional \ref bgl_namedparameters "Named Parameters" described below
 *
 * \cgalNamedParamsBegin
 *   \cgalParamNBegin{verbose}
 *     \cgalParamDescription{indicates whether output warnings and error
 *       messages should be printed.}
 *     \cgalParamType{Boolean}
 *     \cgalParamDefault{`false`}
 *   \cgalParamNEnd
 * \cgalNamedParamsEnd
 *
 * \returns `true` if reading was successful, `false` otherwise.
 *
 * \pre Requires `CGAL_LINKED_WITH_GLTF` to be defined and `tiny_gltf.h` to
 *      be on the include path.
 */
template <typename PointRange, typename PolygonRange,
          typename CGAL_NP_TEMPLATE_PARAMETERS>
bool read_GLTF(std::istream& is,
               PointRange&   points,
               PolygonRange& polygons,
               const CGAL_NP_CLASS& np = parameters::default_values(),
               std::enable_if_t<internal::is_Range<PolygonRange>::value>* = nullptr)
{
  const bool verbose = parameters::choose_parameter(parameters::get_parameter(np, internal_np::verbose), false);

  if(!is.good())
  {
    if(verbose)
      std::cerr << "GLTF: stream is not readable." << std::endl;
    return false;
  }

  // Read entire stream into a byte buffer.
  const std::vector<uint8_t> raw(std::istreambuf_iterator<char>(is),
                                  std::istreambuf_iterator<char>());

  if(raw.empty())
    return true; // empty file — valid, symmetric with write_GLTF

  return internal::read_GLTF_impl(raw, points, polygons, /*base_path=*/"", verbose);
}

/*!
 * \ingroup PkgStreamSupportIoFuncsGLTF
 *
 * \brief reads the content of the file `fname` into `points` and `polygons`,
 *        using the \ref IOStreamGLTF.
 *
 * Both ASCII GLTF (`.gltf`) and binary GLB (`.glb`) files are accepted.
 * The file extension determines the open mode (`std::ios::binary` for `.glb`).
 *
 * \tparam PointRange a model of the concepts `RandomAccessContainer` and
 *         `BackInsertionSequence` whose value type is the point type.
 * \tparam PolygonRange a model of the concept `SequenceContainer` whose
 *         `value_type` is itself a model of `SequenceContainer` whose
 *         `value_type` is an unsigned integer type convertible to `std::size_t`.
 * \tparam NamedParameters a sequence of \ref bgl_namedparameters "Named Parameters"
 *
 * \param fname the path to the input file
 * \param points points of the polygon soup
 * \param polygons a range of triangles
 * \param np optional \ref bgl_namedparameters "Named Parameters" — see
 *           `read_GLTF(std::istream&, ...)` for the list.
 *
 * \returns `true` if reading was successful, `false` otherwise.
 */
template <typename PointRange, typename PolygonRange,
          typename CGAL_NP_TEMPLATE_PARAMETERS>
bool read_GLTF(const std::string& fname,
               PointRange&        points,
               PolygonRange&      polygons,
               const CGAL_NP_CLASS& np = parameters::default_values(),
               std::enable_if_t<internal::is_Range<PolygonRange>::value>* = nullptr)
{
  const bool verbose = parameters::choose_parameter(parameters::get_parameter(np, internal_np::verbose), false);

  // Detect GLB by extension so we can open with the right flags.
  const bool is_glb = fname.size() >= 4 &&
                      fname.compare(fname.size() - 4, 4, ".glb") == 0;

  const auto open_flags = is_glb
                          ? (std::ios::in | std::ios::binary)
                          : std::ios::in;
  std::ifstream file(fname, open_flags);
  if(!file)
  {
    if(verbose)
      std::cerr << "GLTF: cannot open file '" << fname << "'." << std::endl;
    return false;
  }

  const std::vector<uint8_t> raw(std::istreambuf_iterator<char>(file),
                                  std::istreambuf_iterator<char>());

  if(raw.empty())
    return true;

  // Pass the directory of the file as base_path so tinygltf can resolve
  // external buffer URIs (e.g. "mesh.bin").
  const std::string base_path = [&]() -> std::string {
    const auto sep = fname.find_last_of("/\\");
    return sep != std::string::npos ? fname.substr(0, sep + 1) : "";
  }();

  return internal::read_GLTF_impl(raw, points, polygons, base_path, verbose);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
// Write

namespace internal {

// Appends bytes of `value` (little-endian) to `buf`.
template <typename T>
void append_le(std::vector<uint8_t>& buf, T value)
{
  const uint8_t* p = reinterpret_cast<const uint8_t*>(&value);
  buf.insert(buf.end(), p, p + sizeof(T));
}

// Core write function.  Packs points and triangles into a self-contained
// GLTF model (embedded base64 buffer) and writes it to `os`.
// When `binary` is true a GLB stream is written; otherwise ASCII GLTF JSON.
template <typename PointRange, typename PolygonRange>
bool write_GLTF_impl(std::ostream&       os,
                     const PointRange&   points,
                     const PolygonRange& polygons,
                     const bool          binary,
                     const bool          verbose)
{
  if(!os.good())
  {
    if(verbose)
      std::cerr << "GLTF: output stream is not writable." << std::endl;
    return false;
  }

  // --- Build binary payload (positions + indices) ---
  std::vector<uint8_t> pos_buf, idx_buf;

  // Write positions as float triplets.
  double xmin = 1e308, xmax = -1e308;
  double ymin = 1e308, ymax = -1e308;
  double zmin = 1e308, zmax = -1e308;

  for(const auto& pt : points)
  {
    const float x = static_cast<float>(CGAL::to_double(pt.x()));
    const float y = static_cast<float>(CGAL::to_double(pt.y()));
    const float z = static_cast<float>(CGAL::to_double(pt.z()));
    append_le(pos_buf, x);
    append_le(pos_buf, y);
    append_le(pos_buf, z);
    if(x < xmin) xmin = x;  if(x > xmax) xmax = x;
    if(y < ymin) ymin = y;  if(y > ymax) ymax = y;
    if(z < zmin) zmin = z;  if(z > zmax) zmax = z;
  }

  // Write indices as uint32_t for maximum portability.
  std::size_t index_count = 0;
  for(const auto& poly : polygons)
  {
    if(poly.size() < 3) continue;
    // Fan-triangulate in case polygons have >3 vertices.
    for(std::size_t i = 1; i + 1 < poly.size(); ++i)
    {
      append_le(idx_buf, static_cast<uint32_t>(poly[0]));
      append_le(idx_buf, static_cast<uint32_t>(poly[i]));
      append_le(idx_buf, static_cast<uint32_t>(poly[i + 1]));
      index_count += 3;
    }
  }

  if(points.empty() && index_count == 0)
    return true; // nothing to write — valid

  // Pad idx_buf to 4-byte alignment (GLTF spec requirement).
  while(idx_buf.size() % 4 != 0)
    idx_buf.push_back(0x00);

  // --- Assemble tinygltf model ---
  tinygltf::Model model;
  model.asset.version   = "2.0";
  model.asset.generator = "CGAL Stream_support (read_GLTF / write_GLTF)";

  // Single binary buffer: [positions | indices]
  {
    tinygltf::Buffer buf;
    buf.data.insert(buf.data.end(), pos_buf.begin(), pos_buf.end());
    buf.data.insert(buf.data.end(), idx_buf.begin(), idx_buf.end());
    model.buffers.push_back(std::move(buf));
  }

  // Buffer view: positions
  {
    tinygltf::BufferView bv;
    bv.buffer     = 0;
    bv.byteOffset = 0;
    bv.byteLength = pos_buf.size();
    bv.target     = TINYGLTF_TARGET_ARRAY_BUFFER;
    model.bufferViews.push_back(std::move(bv));
  }

  // Buffer view: indices
  {
    tinygltf::BufferView bv;
    bv.buffer     = 0;
    bv.byteOffset = pos_buf.size();
    bv.byteLength = idx_buf.size();
    bv.target     = TINYGLTF_TARGET_ELEMENT_ARRAY_BUFFER;
    model.bufferViews.push_back(std::move(bv));
  }

  // Accessor: positions (VEC3, FLOAT)
  const int pos_acc_idx = static_cast<int>(model.accessors.size());
  {
    tinygltf::Accessor acc;
    acc.bufferView    = 0;
    acc.byteOffset    = 0;
    acc.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
    acc.count         = points.size();
    acc.type          = TINYGLTF_TYPE_VEC3;
    acc.minValues     = {xmin, ymin, zmin};
    acc.maxValues     = {xmax, ymax, zmax};
    model.accessors.push_back(std::move(acc));
  }

  // Accessor: indices (SCALAR, UNSIGNED_INT)
  const int idx_acc_idx = static_cast<int>(model.accessors.size());
  {
    tinygltf::Accessor acc;
    acc.bufferView    = 1;
    acc.byteOffset    = 0;
    acc.componentType = TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT;
    acc.count         = index_count;
    acc.type          = TINYGLTF_TYPE_SCALAR;
    model.accessors.push_back(std::move(acc));
  }

  // Mesh primitive
  tinygltf::Primitive prim;
  prim.attributes["POSITION"] = pos_acc_idx;
  prim.indices = idx_acc_idx;
  prim.mode    = TINYGLTF_MODE_TRIANGLES;

  tinygltf::Mesh mesh;
  mesh.name = "cgal_mesh";
  mesh.primitives.push_back(std::move(prim));
  model.meshes.push_back(std::move(mesh));

  // Node → scene
  tinygltf::Node node;
  node.mesh = 0;
  model.nodes.push_back(std::move(node));

  tinygltf::Scene scene;
  scene.nodes.push_back(0);
  model.scenes.push_back(std::move(scene));
  model.defaultScene = 0;

  // --- Serialise ---
  tinygltf::TinyGLTF writer;
  // embed_images, embed_buffers, pretty_print, write_binary
  const bool pretty    = !binary;
  const bool embed_buf = true;
  const bool embed_img = true;

  // TinyGLTF writes to a file path; we work around this by writing to a
  // temporary string stream and then copying to `os`.
  // WriteGltfSceneToStream is available since tinygltf 2.6.0.
  const bool ok = writer.WriteGltfSceneToStream(&model, os, pretty, binary);

  if(!ok && verbose)
    std::cerr << "GLTF: failed to serialise model." << std::endl;

  return ok;
}

} // namespace internal

// ------------------------------------------------------------------------------------------------
// Public write API
// ------------------------------------------------------------------------------------------------

/*!
 * \ingroup PkgStreamSupportIoFuncsGLTF
 *
 * \brief writes the contents of `points` and `polygons` into `os`, using
 *        the \ref IOStreamGLTF.
 *
 * The output is ASCII GLTF with a self-contained (embedded base64) binary
 * buffer.  Polygon faces with more than three vertices are fan-triangulated.
 *
 * \tparam PointRange a model of the concept `SequenceContainer` whose value
 *         type is the point type.
 * \tparam PolygonRange a model of the concept `SequenceContainer` whose
 *         `value_type` is itself a model of `SequenceContainer` whose
 *         `value_type` is an unsigned integer type convertible to `std::size_t`.
 * \tparam NamedParameters a sequence of \ref bgl_namedparameters "Named Parameters"
 *
 * \param os the output stream
 * \param points points of the polygon soup
 * \param polygons a range of polygons
 * \param np optional \ref bgl_namedparameters "Named Parameters" described below
 *
 * \cgalNamedParamsBegin
 *   \cgalParamNBegin{verbose}
 *     \cgalParamDescription{indicates whether output warnings and error
 *       messages should be printed.}
 *     \cgalParamType{Boolean}
 *     \cgalParamDefault{`false`}
 *   \cgalParamNEnd
 * \cgalNamedParamsEnd
 *
 * \returns `true` if writing was successful, `false` otherwise.
 *
 * \pre Requires `CGAL_LINKED_WITH_GLTF` to be defined and `tiny_gltf.h` to
 *      be on the include path.
 */
template <typename PointRange, typename PolygonRange,
          typename CGAL_NP_TEMPLATE_PARAMETERS>
bool write_GLTF(std::ostream&       os,
                const PointRange&   points,
                const PolygonRange& polygons,
                const CGAL_NP_CLASS& np = parameters::default_values(),
                std::enable_if_t<internal::is_Range<PolygonRange>::value>* = nullptr)
{
  const bool verbose = parameters::choose_parameter(parameters::get_parameter(np, internal_np::verbose), false);
  return internal::write_GLTF_impl(os, points, polygons, /*binary=*/false, verbose);
}

/*!
 * \ingroup PkgStreamSupportIoFuncsGLTF
 *
 * \brief writes the contents of `points` and `polygons` to the file `fname`,
 *        using the \ref IOStreamGLTF.
 *
 * When `fname` ends in `.glb` the output is binary GLB; otherwise ASCII GLTF
 * JSON is written.
 *
 * \tparam PointRange a model of the concept `SequenceContainer` whose value
 *         type is the point type.
 * \tparam PolygonRange a model of the concept `SequenceContainer` whose
 *         `value_type` is itself a model of `SequenceContainer` whose
 *         `value_type` is an unsigned integer type convertible to `std::size_t`.
 * \tparam NamedParameters a sequence of \ref bgl_namedparameters "Named Parameters"
 *
 * \param fname the path to the output file
 * \param points points of the polygon soup
 * \param polygons a range of polygons
 * \param np optional \ref bgl_namedparameters "Named Parameters" — see
 *           `write_GLTF(std::ostream&, ...)` for the list.
 *
 * \returns `true` if writing was successful, `false` otherwise.
 */
template <typename PointRange, typename PolygonRange,
          typename CGAL_NP_TEMPLATE_PARAMETERS>
bool write_GLTF(const std::string&  fname,
                const PointRange&   points,
                const PolygonRange& polygons,
                const CGAL_NP_CLASS& np = parameters::default_values(),
                std::enable_if_t<internal::is_Range<PolygonRange>::value>* = nullptr)
{
  const bool verbose = parameters::choose_parameter(parameters::get_parameter(np, internal_np::verbose), false);

  const bool is_glb = fname.size() >= 4 &&
                      fname.compare(fname.size() - 4, 4, ".glb") == 0;
  const auto open_flags = is_glb
                          ? (std::ios::out | std::ios::binary)
                          : std::ios::out;
  std::ofstream file(fname, open_flags);
  if(!file)
  {
    if(verbose)
      std::cerr << "GLTF: cannot open file '" << fname << "' for writing." << std::endl;
    return false;
  }

  return internal::write_GLTF_impl(file, points, polygons, is_glb, verbose);
}

#endif // CGAL_LINKED_WITH_GLTF || DOXYGEN_RUNNING

} // namespace IO
} // namespace CGAL

#endif // CGAL_IO_GLTF_H
