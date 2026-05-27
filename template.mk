LIBNAME=libtlia
VERSION=1.0.0

TLIA_SRC=$(ROOT)/TLIA/src/vision.cpp
TLIA_OBJ=$(BINDIR)/TLIA/vision.o
TLIA_HEADER=$(ROOT)/TLIA/include/tlia/vision.hpp
TLIA_TEMPLATE=$(ROOT)/$(LIBNAME)-template

.PHONY: library
library: clean $(TLIA_OBJ)
	$(AR) rvs $(BINDIR)/$(LIBNAME).a $(TLIA_OBJ)
	mkdir -p $(TLIA_TEMPLATE)/firmware $(TLIA_TEMPLATE)/include/tlia
	cp $(BINDIR)/$(LIBNAME).a $(TLIA_TEMPLATE)/firmware/$(LIBNAME).a
	cp $(TLIA_HEADER) $(TLIA_TEMPLATE)/include/tlia/vision.hpp
	pros conduct create-template $(TLIA_TEMPLATE) $(LIBNAME) $(VERSION) --system firmware/$(LIBNAME).a --system include/tlia/vision.hpp --target v5
	@echo Created $(TLIA_TEMPLATE)/$(LIBNAME)@$(VERSION).zip

$(TLIA_OBJ): $(TLIA_SRC) $(TLIA_HEADER)
	mkdir -p $(dir $@)
	$(CXX) -c $(INCLUDE) -iquote"$(ROOT)/TLIA/include" $(CXXFLAGS) $(EXTRA_CXXFLAGS) -o $@ $<
