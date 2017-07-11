TARGET =  test


OBJS = main.o LightExtraction.o BlobMatch.o MarkerRecognizer.o reconstuct.o surface_mesh.o
CXX ?= g++

CXXFLAGS += -c -Wall -std=c++11 $(shell pkg-config --cflags opencv) -Iinclude/ \
	-I/usr/include/opencv2/   
LDFLAGS += $(shell pkg-config --libs --static opencv) -lpthread

all: $(TARGET)

$(TARGET):$(OBJS)
	$(CXX) $(OBJS) -o $@ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $< -o $@ $(CXXFLAGS)
	

clean: 
	rm -f $(TARGET)  $(OBJS)  *~ *.ply *.jpg pic/*.jpg lines/* pic/light/*.jpg pic/marker/*.jpg  pic/line/*.jpg
cleanPic:
	rm -f *.jpg *.ply pic/* lines/* pic/light/* pic/marker/* 