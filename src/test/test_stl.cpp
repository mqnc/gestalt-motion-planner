
#include "stl.h"
#include "str.h"
#include "stl_reader_mod.h"
#include "test_main.h"

TEST(test_stl_reader, icosahedra) {
	{
		auto source = str::load("../../../src/test/urdf/ico_binary.stl", true);
		stl_reader::StlMesh <double, unsigned int> mesh(source);
		EXPECT_EQ(mesh.num_vrts(), 12);
		EXPECT_EQ(mesh.num_tris(), 20);
	}
	{
		auto source = str::load("../../../src/test/urdf/ico_ascii.stl", true);
		stl_reader::StlMesh <double, unsigned int> mesh(source);
		EXPECT_EQ(mesh.num_vrts(), 12);
		EXPECT_EQ(mesh.num_tris(), 20);
	}
	{
		auto source = str::load("../../../src/test/urdf/ico_ascii_win.stl", true);
		stl_reader::StlMesh <double, unsigned int> mesh(source);
		EXPECT_EQ(mesh.num_vrts(), 12);
		EXPECT_EQ(mesh.num_tris(), 20);
	}
}

TEST_MAIN
