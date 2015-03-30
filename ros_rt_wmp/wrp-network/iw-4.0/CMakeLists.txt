cmake_minimum_required (VERSION 2.6)
project (Tutorial)

add_library(danilib android-nl.c  coalesce.c  cqm.c   event.c
hwsim.c  info.c       iw.c    mesh.c   mpp.c  offch.c  phy.c  reason.c
roc.c   sections.c  status.c  util.c    version.c
bitrate.c     connect.c   dani.c  genl.c   ibss.c   interface.c  link.c
mpath.c  ocb.c  p2p.c    ps.c   reg.c     scan.c  station.c   survey.c
vendor.c  wowlan.c)

target_link_libraries (danilib nl)

add_executable(daniw EXCLUDE_FROM_ALL dani.c)
target_link_libraries (daniw danilib)
