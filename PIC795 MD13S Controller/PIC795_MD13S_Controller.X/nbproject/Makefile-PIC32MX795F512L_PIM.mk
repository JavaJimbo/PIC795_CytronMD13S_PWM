#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-PIC32MX795F512L_PIM.mk)" "nbproject/Makefile-local-PIC32MX795F512L_PIM.mk"
include nbproject/Makefile-local-PIC32MX795F512L_PIM.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=PIC32MX795F512L_PIM
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/PIC795_MD13S_Controller.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/PIC795_MD13S_Controller.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS

else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../main.c ../DelayPIC32.c ../I2C_4BUS_EEPROM_PIC32.c "../PCA9685 _PIC32.c" ../CRCmodbus.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1472/main.o ${OBJECTDIR}/_ext/1472/DelayPIC32.o ${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o "${OBJECTDIR}/_ext/1472/PCA9685 _PIC32.o" ${OBJECTDIR}/_ext/1472/CRCmodbus.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1472/main.o.d ${OBJECTDIR}/_ext/1472/DelayPIC32.o.d ${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o.d "${OBJECTDIR}/_ext/1472/PCA9685 _PIC32.o.d" ${OBJECTDIR}/_ext/1472/CRCmodbus.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1472/main.o ${OBJECTDIR}/_ext/1472/DelayPIC32.o ${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o ${OBJECTDIR}/_ext/1472/PCA9685\ _PIC32.o ${OBJECTDIR}/_ext/1472/CRCmodbus.o

# Source Files
SOURCEFILES=../main.c ../DelayPIC32.c ../I2C_4BUS_EEPROM_PIC32.c ../PCA9685 _PIC32.c ../CRCmodbus.c



CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-PIC32MX795F512L_PIM.mk dist/${CND_CONF}/${IMAGE_TYPE}/PIC795_MD13S_Controller.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=32MX795F512L
MP_LINKER_FILE_OPTION=
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1472/main.o: ../main.c  .generated_files/75d935c8420a2796dce9b62e441a419b5284f500.flag .generated_files/9485193fe38228777c17feb3d496573bfb48705e.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../Microchip/Include" -DPIC32MX795F512L_PIM -I".." -I"../../Microchip/Include" -MP -MMD -MF "${OBJECTDIR}/_ext/1472/main.o.d" -o ${OBJECTDIR}/_ext/1472/main.o ../main.c    -DXPRJ_PIC32MX795F512L_PIM=$(CND_CONF)    $(COMPARISON_BUILD)    
	
${OBJECTDIR}/_ext/1472/DelayPIC32.o: ../DelayPIC32.c  .generated_files/700e22c4598766a980c6d4108c8bcd91b6167a62.flag .generated_files/9485193fe38228777c17feb3d496573bfb48705e.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/DelayPIC32.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/DelayPIC32.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../Microchip/Include" -DPIC32MX795F512L_PIM -I".." -I"../../Microchip/Include" -MP -MMD -MF "${OBJECTDIR}/_ext/1472/DelayPIC32.o.d" -o ${OBJECTDIR}/_ext/1472/DelayPIC32.o ../DelayPIC32.c    -DXPRJ_PIC32MX795F512L_PIM=$(CND_CONF)    $(COMPARISON_BUILD)    
	
${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o: ../I2C_4BUS_EEPROM_PIC32.c  .generated_files/495867b5bac55323cdba065c119c1fec2bf321f3.flag .generated_files/9485193fe38228777c17feb3d496573bfb48705e.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../Microchip/Include" -DPIC32MX795F512L_PIM -I".." -I"../../Microchip/Include" -MP -MMD -MF "${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o.d" -o ${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o ../I2C_4BUS_EEPROM_PIC32.c    -DXPRJ_PIC32MX795F512L_PIM=$(CND_CONF)    $(COMPARISON_BUILD)    
	
${OBJECTDIR}/_ext/1472/PCA9685\ _PIC32.o: ../PCA9685\ _PIC32.c  .generated_files/c590df2969153f282a22702d7c0af98797514a4d.flag .generated_files/9485193fe38228777c17feb3d496573bfb48705e.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} "${OBJECTDIR}/_ext/1472/PCA9685 _PIC32.o".d 
	@${RM} "${OBJECTDIR}/_ext/1472/PCA9685 _PIC32.o" 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../Microchip/Include" -DPIC32MX795F512L_PIM -I".." -I"../../Microchip/Include" -MP -MMD -MF "${OBJECTDIR}/_ext/1472/PCA9685 _PIC32.o.d" -o "${OBJECTDIR}/_ext/1472/PCA9685 _PIC32.o" "../PCA9685 _PIC32.c"    -DXPRJ_PIC32MX795F512L_PIM=$(CND_CONF)    $(COMPARISON_BUILD)    
	
${OBJECTDIR}/_ext/1472/CRCmodbus.o: ../CRCmodbus.c  .generated_files/d8851f8075ca6dde63ee000d16408d495666916d.flag .generated_files/9485193fe38228777c17feb3d496573bfb48705e.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/CRCmodbus.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/CRCmodbus.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../Microchip/Include" -DPIC32MX795F512L_PIM -I".." -I"../../Microchip/Include" -MP -MMD -MF "${OBJECTDIR}/_ext/1472/CRCmodbus.o.d" -o ${OBJECTDIR}/_ext/1472/CRCmodbus.o ../CRCmodbus.c    -DXPRJ_PIC32MX795F512L_PIM=$(CND_CONF)    $(COMPARISON_BUILD)    
	
else
${OBJECTDIR}/_ext/1472/main.o: ../main.c  .generated_files/4cbcf7c07e1af9ec3cb6e83d13257108e77ce045.flag .generated_files/9485193fe38228777c17feb3d496573bfb48705e.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../Microchip/Include" -DPIC32MX795F512L_PIM -I".." -I"../../Microchip/Include" -MP -MMD -MF "${OBJECTDIR}/_ext/1472/main.o.d" -o ${OBJECTDIR}/_ext/1472/main.o ../main.c    -DXPRJ_PIC32MX795F512L_PIM=$(CND_CONF)    $(COMPARISON_BUILD)    
	
${OBJECTDIR}/_ext/1472/DelayPIC32.o: ../DelayPIC32.c  .generated_files/a5c323c7320c8c8f535b3cbf6de6261a71b5f39c.flag .generated_files/9485193fe38228777c17feb3d496573bfb48705e.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/DelayPIC32.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/DelayPIC32.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../Microchip/Include" -DPIC32MX795F512L_PIM -I".." -I"../../Microchip/Include" -MP -MMD -MF "${OBJECTDIR}/_ext/1472/DelayPIC32.o.d" -o ${OBJECTDIR}/_ext/1472/DelayPIC32.o ../DelayPIC32.c    -DXPRJ_PIC32MX795F512L_PIM=$(CND_CONF)    $(COMPARISON_BUILD)    
	
${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o: ../I2C_4BUS_EEPROM_PIC32.c  .generated_files/8573cb40e36e13253c95b65878b369e163d2cad3.flag .generated_files/9485193fe38228777c17feb3d496573bfb48705e.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../Microchip/Include" -DPIC32MX795F512L_PIM -I".." -I"../../Microchip/Include" -MP -MMD -MF "${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o.d" -o ${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o ../I2C_4BUS_EEPROM_PIC32.c    -DXPRJ_PIC32MX795F512L_PIM=$(CND_CONF)    $(COMPARISON_BUILD)    
	
${OBJECTDIR}/_ext/1472/PCA9685\ _PIC32.o: ../PCA9685\ _PIC32.c  .generated_files/259a026bd74f35201eb444548023025361123e24.flag .generated_files/9485193fe38228777c17feb3d496573bfb48705e.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} "${OBJECTDIR}/_ext/1472/PCA9685 _PIC32.o".d 
	@${RM} "${OBJECTDIR}/_ext/1472/PCA9685 _PIC32.o" 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../Microchip/Include" -DPIC32MX795F512L_PIM -I".." -I"../../Microchip/Include" -MP -MMD -MF "${OBJECTDIR}/_ext/1472/PCA9685 _PIC32.o.d" -o "${OBJECTDIR}/_ext/1472/PCA9685 _PIC32.o" "../PCA9685 _PIC32.c"    -DXPRJ_PIC32MX795F512L_PIM=$(CND_CONF)    $(COMPARISON_BUILD)    
	
${OBJECTDIR}/_ext/1472/CRCmodbus.o: ../CRCmodbus.c  .generated_files/add1df57eeeddc7e75ceae69f10fdf531a6d597f.flag .generated_files/9485193fe38228777c17feb3d496573bfb48705e.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/CRCmodbus.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/CRCmodbus.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../Microchip/Include" -DPIC32MX795F512L_PIM -I".." -I"../../Microchip/Include" -MP -MMD -MF "${OBJECTDIR}/_ext/1472/CRCmodbus.o.d" -o ${OBJECTDIR}/_ext/1472/CRCmodbus.o ../CRCmodbus.c    -DXPRJ_PIC32MX795F512L_PIM=$(CND_CONF)    $(COMPARISON_BUILD)    
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/PIC795_MD13S_Controller.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE) -g -mdebugger -D__MPLAB_DEBUGGER_ICD3=1 -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/PIC795_MD13S_Controller.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_PIC32MX795F512L_PIM=$(CND_CONF)    $(COMPARISON_BUILD)   -mreserve=data@0x0:0x1FC -mreserve=boot@0x1FC02000:0x1FC02FEF -mreserve=boot@0x1FC02000:0x1FC024FF  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD3=1,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map" 
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/PIC795_MD13S_Controller.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/PIC795_MD13S_Controller.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_PIC32MX795F512L_PIM=$(CND_CONF)    $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map" 
	${MP_CC_DIR}\\xc32-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/PIC795_MD13S_Controller.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/PIC32MX795F512L_PIM
	${RM} -r dist/PIC32MX795F512L_PIM

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
