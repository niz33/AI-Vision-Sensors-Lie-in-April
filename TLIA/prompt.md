currently the repo contains code for a vex robot using the pros library. in the arc/vision_algo.cpp, there are two algorithms for computing the location of a camera relative to a tag.

Now we want to refactor the testing code into a proper library within the folder TLIA

too make a pros library here is a documentation provided:

```
Making Libraries for PROS using GitHub Releases

Make a template.mk file in the root directory of your project and paste the following template:

LIBNAME=libfbc
VERSION=1.0.0

# extra files (like header files)
TEMPLATEFILES = include/fbc_pid.h include/fbc.h
# basename of the source files that should be archived
TEMPLATEOBJS = fbc_pid fbc

TEMPLATE=$(ROOT)/$(LIBNAME)-template

.DEFAULT_GOAL: all

library: clean $(BINDIR) $(SUBDIRS) $(ASMOBJ) $(COBJ) $(CPPOBJ)
    $(MCUPREFIX)ar rvs $(BINDIR)/$(LIBNAME).a $(foreach f,$(TEMPLATEOBJS),$(BINDIR)/$(f).o)
    mkdir -p $(TEMPLATE) $(TEMPLATE)/firmware $(addprefix $(TEMPLATE)/, $(dir $(TEMPLATEFILES)))
    cp $(BINDIR)/$(LIBNAME).a $(TEMPLATE)/firmware/$(LIBNAME).a
    $(foreach f,$(TEMPLATEFILES),cp $(f) $(TEMPLATE)/$(f);)
    pros conduct create-template $(LIBNAME) $(VERSION) $(TEMPLATE) --ignore template.pros --upgrade-files firmware/$(LIBNAME).a $(foreach f,$(TEMPLATEFILES),--upgrade-files $(f))
    @echo Need to zip $(TEMPLATE) without the base directory
    # cd $(TEMPLATE) && zip -r ../$(basename $(TEMPLATE)) *

You should change LIBNAME, VERSION, TEMPLATEFILES, and TEMPLATEOBJS to fit your project.

In the project’s Makefile, add the following line to line 14:

-include $(ROOT)/template.mk

Then to build the library, run pros make library. Next, you will need to zip the template directory. The zip file should not contain the libfbc-template directory (that is, the root of the zip file should contain template.pros and all your other files). Next, you should create a release on GitHub and upload your template(s) to the release. You can see Purdue SIGBots’ repository at purduesigbots/libblrs.
```

The library should be a state machine, that is, it is an object that
- asks for pointers to ai vision sensors and their positions (make a struct that groups the info of the pointer and position and rotation). The ai vision sensors are however assumed to be vertical and the camera view is in the default rotation.
- can be called to calculate the position of the robot relative to a tag of a given id and the robots relative rotation to that id. for on the calculation later.
- 
