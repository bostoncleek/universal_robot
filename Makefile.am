ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -I$(top_srcdir)/include

EXTRA_DIST =
bin_PROGRAMS =
bin_SCRIPTS =
lib_LTLIBRARIES =

ur5_robotiq_robot_limited.urdf: ur_description/urdf/ur5_robotiq_robot_limited.urdf.xacro
	ROS_PACKAGE_PATH="$(ROS_PACKAGE_PATH):$(top_srcdir)" \
		rosrun xacro xacro -o $@ $<

scene.c: ur5_robotiq_robot_limited.urdf
	ROS_PACKAGE_PATH="$(ROS_PACKAGE_PATH):$(top_srcdir)" \
	$(AARXC) ur5_robotiq_robot_limited.urdf \
		-o $@ -n "kavrakilab"

lib_LTLIBRARIES += libkavrakilab-scene.la
libkavrakilab_scene_la_SOURCES = scene.c
libkavrakilab_scene_la_LDFLAGS = -avoid-version -module -shared -export-dynamic
