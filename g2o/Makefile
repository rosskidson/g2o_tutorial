all: g2o_built

SOURCE_DIR = g2o_src
TOP_DIR = $(PWD)
EIGEN_DIR = $(shell rospack find eigen)/include

g2o_built:
	-rm -rf lib include bin
	-mkdir -p $(SOURCE_DIR)/build
	cd $(SOURCE_DIR)/build && cmake ../ -DCMAKE_INSTALL_PREFIX=$(TOP_DIR) -DG2O_EIGEN3_INCLUDE=$(EIGEN_DIR) && make -j2 install
	cp $(SOURCE_DIR)/lib/libg2o_tutorial_slam2d.so lib
	touch g2o_built

clean:
	rm -rf lib bin include $(SOURCE_DIR)/build $(SOURCE_DIR)/bin $(SOURCE_DIR)/lib
	rm -rf g2o_built
	rm -rf g2o # old target
