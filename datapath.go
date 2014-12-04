// Code written by Chris Stiteler for CS472 (thursday)
// Project 3 - Due 12/04/2014
package main

import (
	"fmt"
	"strings"
)

const (
	// function codes OR opcode values
	ADD = 0x20
	NOP = 0x0
	SUB = 0x22
	LB  = 0x20
	SB  = 0x28
)

// Memory
type Memory [1024]byte

var Main_Memory Memory

// Normal Registers
type Registers [32]int32

var Regs Registers

// "program counter" value (index into instruction cache)
var PC int

// our raw hex instruction input
var InstructionCache = []uint32{}

// the Disassembleable interface is fulfilled by
// both R and I format MIPS instructions, implicitly
type Disassembleable interface {
	disassemble()
	isRFormat() bool
}

// Pipeline Registers
type IFIDReg struct {
	// the instruction itself
	Inst uint32
}

// our Write/Read IFID Registers
var W_IFID IFIDReg
var R_IFID IFIDReg

// our ID/EX register, written to by ID stage
type IDEXReg struct {
	//controls:
	RegDst   bool
	ALUSrc   bool
	ALUOp    uint32
	MemRead  bool
	MemWrite bool
	Branch   bool
	MemToReg bool
	RegWrite bool
	// IRL func bits would wire direct to ALUControl from ID stage
	RFunctionBits uint32

	// values
	ReadReg1Value  int32
	ReadReg2Value  int32
	SEOffset       int32
	WriteRegNumR   uint32
	WriteRegNumAlt uint32
}

// our Write/Read IDEX Registers
var W_IDEX IDEXReg
var R_IDEX IDEXReg

type EXMEMReg struct {
	// controls:
	MemRead  bool
	MemWrite bool
	Branch   bool
	MemToReg bool
	RegWrite bool

	// values
	isZero      bool
	ALUResult   int32
	SWValue     int32
	WriteRegNum uint32
}

// our Write/Read EX/MEM Registers
var W_EXMEM EXMEMReg
var R_EXMEM EXMEMReg

type MEMWBReg struct {
	// controls:
	MemToReg bool
	RegWrite bool

	// values
	LBDataValue byte
	ALUResult   int32
	WriteRegNum uint32
}

// our Write/Read MEM/WB Registers
var W_MEMWB MEMWBReg
var R_MEMWB MEMWBReg

// for printing
var ClockCycle int

func main() {
	fmt.Println("Code written by Chris Stiteler for CS472 (thursday)\nProject 3 - Due 12/04/2014\n")
	initialize()
	ClockCycle = 0
	// clock cycle 0 state check
	print()

	// run the pipeline
	for i := 0; i < len(InstructionCache); i++ {
		ClockCycle++
		IF_Stage()
		ID_Stage()
		EX_Stage()
		MEM_Stage()
		WB_Stage()
		print()
		advanceRegisters()
	}
}

func initialize() {
	// init main memory
	initMainMemory()

	// init pipeline registers
	initPipelineRegs()

	// init normal registers
	initRegs()

	// init instructions
	initInstructionCache()
}

//// PIPELINE STAGES:

// Instruction Fetch
func IF_Stage() {
	// simply put the current instruction in the WRITE side IF/ID reg
	W_IFID.Inst = InstructionCache[PC]
	PC++
}

// Decode the instruction, pass information to write ID/EX register
func ID_Stage() {
	// read instruction from read IF/ID reg side
	instructionBits := R_IFID.Inst

	// decode instruction
	instruction := decodeInstruction(instructionBits)
	instruction.disassemble()

	// set controls
	if instruction.isRFormat() {
		// for all rtype instructions the following apply
		W_IDEX.RegDst = true
		W_IDEX.ALUSrc = false
		W_IDEX.RegWrite = true
		W_IDEX.MemToReg = false
		W_IDEX.MemRead = false
		W_IDEX.MemWrite = false
		W_IDEX.Branch = false

		// ALUOp of 10 (in binary) signals an arithmetic operation
		W_IDEX.ALUOp = 2
		// functionality depends on the function bits (in EX stage)
		W_IDEX.RFunctionBits = instruction.(*rInstruction).function

	} else if instruction.(*iInstruction).opcode == LB {
		// set up control sigs for load byte
		// destination bits are 16-20
		W_IDEX.RegDst = false
		W_IDEX.ALUSrc = true
		W_IDEX.RegWrite = true
		W_IDEX.MemToReg = true
		W_IDEX.MemRead = true
		W_IDEX.MemWrite = false
		W_IDEX.Branch = false
		W_IDEX.ALUOp = 0
		// we don't care here, could just grab the junk bits, but want to keep printout clean
		W_IDEX.RFunctionBits = 0x0

		nonSignExtendedOffset := maskAndShiftShort(iConstMask, int16(instructionBits))
		W_IDEX.SEOffset = int32(nonSignExtendedOffset)
	} else if instruction.(*iInstruction).opcode == SB {
		// store byte control signals
		W_IDEX.ALUSrc = true
		W_IDEX.RegWrite = false
		W_IDEX.MemRead = false
		W_IDEX.MemWrite = true
		W_IDEX.Branch = false
		W_IDEX.ALUOp = 0
		// we don't care here, could just grab the junk bits, but want to keep printout clean
		W_IDEX.RFunctionBits = 0x0

		nonSignExtendedOffset := maskAndShiftShort(iConstMask, int16(instructionBits))
		W_IDEX.SEOffset = int32(nonSignExtendedOffset)
	}

	// set values
	W_IDEX.ReadReg1Value = Regs[int(maskAndShift(src1Mask, instructionBits))]
	W_IDEX.ReadReg2Value = Regs[int(maskAndShift(src2Mask, instructionBits))]

	// for rType instructions, WriteRegNum is bits 15-11
	W_IDEX.WriteRegNumR = maskAndShift(rDestMask, instructionBits)
	// for iType Instructions, writeRegNum is bits 20-16 (normally the src2 mask)
	W_IDEX.WriteRegNumAlt = maskAndShift(src2Mask, instructionBits)

}

// Execute the instruction and place info in the EX/MEM Pipeline Register
func EX_Stage() {

	// simulating the ALUSrc MUX
	var inputToALU int32
	if R_IDEX.ALUSrc {
		// if ALUSrc is true, our input to ALU is SEOffset and Reg1
		inputToALU = R_IDEX.SEOffset
	} else {
		// else our input to ALU is reg1, reg2 values
		inputToALU = R_IDEX.ReadReg2Value
	}

	// need to switch on ALUOp: 10 (bin) for arithmetic, 00 for lb/sb
	switch R_IDEX.ALUOp {
	case 2:
		// we have an arith. op (add, sub, or nop)
		switch R_IDEX.RFunctionBits {
		case ADD:
			W_EXMEM.ALUResult = R_IDEX.ReadReg1Value + inputToALU
			break
		case SUB:
			W_EXMEM.ALUResult = R_IDEX.ReadReg1Value - inputToALU
			break
		case NOP:
			return
		}
		W_EXMEM.WriteRegNum = R_IDEX.WriteRegNumR
	case 0:
		// we have a lb/sb to process
		if R_IDEX.MemToReg {
			// we're loading a byte
			// adds the SEOffset to the register
			W_EXMEM.ALUResult = inputToALU + R_IDEX.ReadReg1Value

			// write to register for a lb
			W_EXMEM.WriteRegNum = R_IDEX.WriteRegNumAlt

		} else if R_IDEX.MemWrite {
			// we're storing a byte:
			W_EXMEM.ALUResult = inputToALU + R_IDEX.ReadReg1Value
			//W_EXMEM.SWValue = R_IDEX.ReadReg2Value
		}
	default:
		fmt.Println("Error reading ALUOp")
	}

	// A real datapath would always just do this, even if not needed
	W_EXMEM.SWValue = R_IDEX.ReadReg2Value

	// don't really need, for branches
	if W_EXMEM.ALUResult != 0 {
		W_EXMEM.isZero = false
	}

	// forward other values to write side EX/MEM Register
	W_EXMEM.MemRead = R_IDEX.MemRead
	W_EXMEM.MemWrite = R_IDEX.MemWrite
	W_EXMEM.Branch = R_IDEX.Branch
	W_EXMEM.MemToReg = R_IDEX.MemToReg
	W_EXMEM.RegWrite = R_IDEX.RegWrite
}

// Interact with Memory if necessary (lb/sb), otherwise, just forward controls
func MEM_Stage() {
	// if instruction is a lb, use address calculated in EX, index into MM
	if R_EXMEM.MemRead {
		// index into MM (think we use the ALU result for this)
		W_MEMWB.LBDataValue = Main_Memory[R_EXMEM.ALUResult]
	} else if R_EXMEM.MemWrite {
		// we're doing a store byte
		// THIS WILL TRUNCATE THE BITS IN THE REGISTER (12 bits)
		Main_Memory[R_EXMEM.ALUResult] = byte(R_EXMEM.SWValue)
	}
	// then forward control information from EX/MEM to MEM/WB registers
	W_MEMWB.ALUResult = R_EXMEM.ALUResult
	W_MEMWB.MemToReg = R_EXMEM.MemToReg
	W_MEMWB.RegWrite = R_EXMEM.RegWrite
	W_MEMWB.WriteRegNum = R_EXMEM.WriteRegNum
}

// Write back to register if necessary
func WB_Stage() {
	if R_MEMWB.RegWrite {
		if R_MEMWB.MemToReg {
			// if doing load byte from memory:
			Regs[R_MEMWB.WriteRegNum] = int32(R_MEMWB.LBDataValue)
		} else {
			// else we're doing a simple R instruction:
			Regs[R_MEMWB.WriteRegNum] = R_MEMWB.ALUResult
		}
	}

}

// print the Regs values and the R/W versions of each pipeline reg.
func print() {
	thickLine := "==========================================================="
	thinLine := "-----------------------------------------------------------"

	fmt.Println(thickLine)
	fmt.Println("CLOCK CYCLE: ", ClockCycle, "\n")

	// Normal Registers
	fmt.Println(Regs)
	fmt.Println(thinLine)

	// write then read versions of each PipeReg
	fmt.Printf("WRITE IF/ID REG:\n %+v\n\n", W_IFID)
	fmt.Printf("READ IF/ID REG:\n %+v\n\n", R_IFID)
	fmt.Println(thinLine)

	fmt.Printf("Write ID/EX Reg: %+v\n\n", W_IDEX)
	fmt.Printf("Read  ID/EX Reg: %+v\n\n", R_IDEX)
	fmt.Println(thinLine)

	fmt.Printf("Write EX/MEM Reg: %+v\n\n", W_EXMEM)
	fmt.Printf("Read  EX/MEM Reg: %+v\n\n", R_EXMEM)
	fmt.Println(thinLine)

	fmt.Printf("Write MEM/WB Reg: \n%+v\n\n", W_MEMWB)
	fmt.Printf("Read  MEM/WB Reg: \n%+v\n\n", R_MEMWB)
	fmt.Println(thinLine)

}

func advanceRegisters() {
	// copy all write registers to their read counterparts
	R_IFID = W_IFID
	R_IDEX = W_IDEX
	R_EXMEM = W_EXMEM
	R_MEMWB = W_MEMWB
}

//// INITIALIZATION FUNCTIONS
func initMainMemory() {
	inc := byte(0x00)
	for i, _ := range Main_Memory {
		Main_Memory[i] = inc
		if inc == 0xFF {
			inc = 0x00
			continue
		}
		inc++
	}
}

func initPipelineRegs() {
	// all registers by default will decode as NOPS

	// init the write versions
	W_IFID = IFIDReg{}
	W_IDEX = IDEXReg{}
	W_EXMEM = EXMEMReg{}
	W_MEMWB = MEMWBReg{}

	// init the read versions
	R_IFID = IFIDReg{}
	R_IDEX = IDEXReg{}
	R_EXMEM = EXMEMReg{}
	R_MEMWB = MEMWBReg{}
}

func initRegs() {
	// ensure register 0 contains 0x0
	Regs[0] = 0x0

	// then every other register
	for i := int32(1); i < 32; i++ {
		Regs[i] = 0x100 + i
	}
}

func initInstructionCache() {
	// our input instructions
	InstructionCache = []uint32{
		0xA1020000,
		0x810AFFFC,
		0x00831820,
		0x01263820,
		0x01224820,
		0x81180000,
		0x81510010,
		0x00624022,
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,
	}

	/*
		for ref, the instructions are as follows:
		sb $2, 0 ($8)
		lb $10, -4 ($8)
		add $3, $4, $3
		add $7, $9, $6
		add $9, $9, $2
		lb $24, 0 ($8)
		lb $17, 16 ($10)
		sub $8, $3, $2
		nop x4
	*/

	// start "program counter" at 0
	PC = 0
}

//// STRING METHODS
func (r Registers) String() string {
	registerStrings := make([]string, len(r)+1)
	for i, _ := range r {
		if i == 0 {
			registerStrings[i] = fmt.Sprintf("REGISTERS:\n[$%.2d]: 0x%.3X\n", i, r[i])
		} else {
			registerStrings[i] = fmt.Sprintf("[$%.2d]: 0x%.3X\n", i, r[i])
		}
	}
	return fmt.Sprintf(strings.Join(registerStrings, ""))
}

func (r IFIDReg) String() string {
	return fmt.Sprintf("Instruction: 0x%X", r.Inst)
}

func (r IDEXReg) String() string {
	formatString := "\n[RegDst: %v], [ALUSrc: %v], [ALUOp: 0x%X]\n[MemRead: %v], [MemWrite: %v], [MemToReg: %v]\n[Branch: %v],[RegWrite: %v]\nRFunctionBits: 0x%X\nReadReg1Value: 0x%X\nReadReg2Value: 0x%X\nSEOffset: 0x%X\nWriteRegNumR: %d\nWriteRegNumAlt:%d"
	return fmt.Sprintf(formatString, r.RegDst, r.ALUSrc, r.ALUOp, r.MemRead, r.MemWrite, r.MemToReg,
		r.Branch, r.RegWrite, r.RFunctionBits, r.ReadReg1Value, r.ReadReg2Value,
		r.SEOffset, r.WriteRegNumR, r.WriteRegNumAlt)
}

func (r EXMEMReg) String() string {
	formatString := "\n[MemRead: %v], [MemWrite: %v], [MemToReg: %v]\n[Branch: %v],[RegWrite: %v]\nisZero: %v\nALUResult: 0x%X\nSWValue: 0x%X\nWriteRegNum:%d"
	return fmt.Sprintf(formatString, r.MemRead, r.MemWrite, r.MemToReg,
		r.Branch, r.RegWrite, r.isZero, r.ALUResult,
		r.SWValue, r.WriteRegNum)
}

func (r MEMWBReg) String() string {
	formatString := "[MemToReg: %v], [RegWrite: %v]\nLBDataValue 0x%X\nALUResult: 0x%X\nWriteRegNum: %d"
	return fmt.Sprintf(formatString, r.MemToReg, r.RegWrite, r.LBDataValue, r.ALUResult, r.WriteRegNum)
}

//// from Disassemble
// rInstruction represents an r-format MIPS instruction
type rInstruction struct {
	bits uint32
	//address     uint32
	instruction string
	opcode      uint32
	src1        uint32
	src2        uint32
	dest        uint32
	function    uint32
}

// iInstruction represents an i-format MIPS instruction
type iInstruction struct {
	bits uint32
	//address     uint32
	instruction string
	opcode      uint32
	src1        uint32
	src2        uint32
	constant    int16
}

type Mask struct {
	bits  uint32
	shift uint8
}

// important instruction masks and their subsequent shift values
var opcodeMask = Mask{0xFC000000, 26}
var src1Mask = Mask{0x03E00000, 21}
var src2Mask = Mask{0x001F0000, 16}
var rDestMask = Mask{0x0000F800, 11}
var rFuncMask = Mask{0x0000003F, 0}
var iConstMask = Mask{0x0000FFFF, 0}

func decodeInstruction(input uint32) Disassembleable {
	var instruction Disassembleable

	opcode := maskAndShift(opcodeMask, input)

	// do we need the address?
	// need to get opcode, if 000000, build an rInstruction, else iInstruction
	if opcode == 0 {
		instruction = &rInstruction{bits: input, opcode: opcode}
	} else {
		instruction = &iInstruction{bits: input, opcode: opcode}
	}

	return instruction
}

// r.disassemble() extracts data from rInstruction bits
// also fulfills the Disassembleable interface for rInstructs.
func (r *rInstruction) disassemble() {
	r.src1 = maskAndShift(src1Mask, r.bits)
	r.src2 = maskAndShift(src2Mask, r.bits)
	r.dest = maskAndShift(rDestMask, r.bits)
	r.function = maskAndShift(rFuncMask, r.bits)
}

func (r *rInstruction) isRFormat() bool {
	return true
}

// i.dissassemble() extracts data from iInstruction bits
// also fulfills the Disassembleable interface for iInstructs.
func (i *iInstruction) disassemble() {
	i.src1 = maskAndShift(src1Mask, i.bits)
	i.src2 = maskAndShift(src2Mask, i.bits)
	i.constant = maskAndShiftShort(iConstMask, int16(i.bits))
}

func (i *iInstruction) isRFormat() bool {
	return false
}

// maskAndShift() returns desired bits in a 32-bit value
// depending on the mask (including a shift value)
func maskAndShift(mask Mask, inputBits uint32) uint32 {
	return (inputBits & mask.bits) >> mask.shift
}

// maskAndShift() returns desired bits in a 16-bit value
// depending on the mask (including a shift value)
func maskAndShiftShort(mask Mask, inputBits int16) int16 {
	return (inputBits & int16(mask.bits)) >> mask.shift
}
