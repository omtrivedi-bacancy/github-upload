 #
 #############################################################################
 # Copyright by ams AG                                                       #
 # All rights are reserved.                                                  #
 #                                                                           #
 # IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     #
 # THE SOFTWARE.                                                             #
 #                                                                           #
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       #
 # "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         #
 # LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         #
 # FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  #
 # OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     #
 # SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          #
 # LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     #
 # DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     #
 # THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       #
 # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     #
 # OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      #
 #############################################################################
 #/

#library name and version
LIB_MAJOR		:= 0
LIB_MINOR		:= 0
LIB_BUILD		:= 2
LIB_NAME		:= libas7050_chiplib

#the sources for the library
SRCS			:= $(wildcard ./*.c)
OBJS			:= $(patsubst %.c,%.o,$(SRCS))

#library objects to create
LIB_SHBASENAME	:= $(LIB_NAME).so
LIB_SHSONAME	:= $(LIB_SHBASENAME).$(LIB_MAJOR)
LIB_SHLIBNAME	:= $(LIB_SHBASENAME).$(LIB_MAJOR).$(LIB_MINOR).$(LIB_BUILD)
LIB_STLIBNAME	:= $(LIB_NAME).a

#include directories
EXTRA_INC		:= $(abspath .)
LIB_INC			:= 
STATIC_OBJS		:= 

#additional compiler settings
SHARED_OBJS		:= 
CFLAGS 			:= -fPIE -fPIC $(addprefix -I, $(EXTRA_INC)) -DSENSOR_DEVICE_PATH=\"/dev/as7050\"
LDFLAGS 		:= $(LIB_INC)

#output directory for the library
OUTPUT_DIR		:= $(abspath ./lib)

#commands used in this makefile
LN 				:= ln -sf
MKDIR			:= mkdir 

#the makefile targets
.PHONY: all clean outDir checkCompiler

all: checkCompiler outDir $(LIB_STLIBNAME) $(LIB_SHLIBNAME) $(LIB_SHSONAME) $(LIB_SHBASENAME)

checkCompiler:
ifeq ($(shell which $(CC)), )
	@echo "Compiler not found! Please define!"
endif

outDir:
ifeq ($(wildcard $(OUTPUT_DIR)), )
	$(MKDIR) $(OUTPUT_DIR)
endif

$(LIB_SHLIBNAME): $(OBJS)
	$(CC) $(CFLAGS) -shared $(LDFLAGS) -Wl,-soname,$(LIB_SHSONAME) \
		-o $(OUTPUT_DIR)/$@ $^ $(SHARED_OBJS) -lc -lm

$(LIB_SHSONAME):
	$(LN) $(OUTPUT_DIR)/$(LIB_SHLIBNAME) $(OUTPUT_DIR)/$@

$(LIB_SHBASENAME):
	$(LN) $(OUTPUT_DIR)/$(LIB_SHLIBNAME) $(OUTPUT_DIR)/$@

$(LIB_STLIBNAME): $(OBJS) $(STATIC_OBJS)
	$(AR) rcvs $(OUTPUT_DIR)/$@ $^

%.o: %.c
	$(CC) $(CFLAGS) -c $^ -o $@
	
clean:
	$(RM) $(OBJS)
	$(RM) $(OUTPUT_DIR)/$(LIB_SHLIBNAME)
	$(RM) $(OUTPUT_DIR)/$(LIB_SHSONAME)
	$(RM) $(OUTPUT_DIR)/$(LIB_SHBASENAME)
	$(RM) $(OUTPUT_DIR)/$(LIB_STLIBNAME)

#eof
