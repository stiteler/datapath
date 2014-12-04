// Code written by Chris Stiteler for CS472 (thursday)
// Project 3 - Due 12/04/2014
package main

import (
	"fmt"
	"strings"
)

const (
	// functions OR opcode values
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
type Registers [32]uint32

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

// our two IFID Registers
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

	// these would go direct to ALUControl from InstructionDecode stage,
	// but I figured I coudl put them here
	RFunctionBits uint32

	// values
	ReadReg1Value uint32
	ReadReg2Value uint32
	SEOffset      int32

	WriteRegNumR   uint32
	WriteRegNumAlt uint32
}

// our two IDEX Registers
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
	// CalcBTA branch target offset?
	isZero      bool
	ALUResult   int32
	SWValue     uint32
	WriteRegNum uint32
}

// our two EXMEM Registers
var W_EXMEM EXMEMReg
var R_EXMEM EXMEMReg

type MEMWBReg struct {
	// controls:
	MemToReg bool
	RegWrite bool

	LBDataValue byte
	ALUResult   int32
	WriteRegNum uint32
}

var W_MEMWB MEMWBReg
var R_MEMWB MEMWBReg

func main() {
	initialize()

	// while not done
	for i := 0; i < len(InstructionCache); i++ {
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

func IF_Stage() {
	// simply put the current instruction in the WRITE side IF/ID reg
	W_IFID.Inst = InstructionCache[PC]
	PC++
}

func ID_Stage() {
	// read instruction from read IF/ID side
	instructionBits := R_IFID.Inst
	//fmt.Printf("Instruction bits: 0x%x\n", R_IFID.Inst)

	// decode instruction
	instruction := decodeInstruction(instructionBits)
	instruction.disassemble()

	// set controls
	if instruction.isRFormat() {
		// TODO make sure i've got the regdst issue aligned with example

		// set control bits for all rtype instructions
		W_IDEX.RegDst = true
		W_IDEX.ALUSrc = false
		W_IDEX.RegWrite = true
		W_IDEX.MemToReg = false
		W_IDEX.MemRead = false
		W_IDEX.MemWrite = false
		W_IDEX.Branch = false

		// depending on instruction function
		//rinstr := instruction.(rInstruction)
		
		// ALUOp of 10 (bits) signals an arithmetic operation
		W_IDEX.ALUOp = 2
		W_IDEX.RFunctionBits = instruction.(*rInstruction).function


		//fmt.Printf("W_ID/EX: %+v\n\n", W_IDEX)
	} else if instruction.(*iInstruction).opcode == 0x20 {
		// load byte
		// destination bits are 16-20
		W_IDEX.RegDst = false
		// we're using offset value in ALU
		W_IDEX.ALUSrc = true
		W_IDEX.RegWrite = true
		W_IDEX.MemToReg = true
		W_IDEX.MemRead = true
		W_IDEX.MemWrite = false
		W_IDEX.Branch = false
		W_IDEX.ALUOp = 0
	} else {
		// store byte
		W_IDEX.ALUSrc = true
		W_IDEX.RegWrite = false
		W_IDEX.MemRead = true
		W_IDEX.MemWrite = true
		W_IDEX.Branch = false
		W_IDEX.ALUOp = 0
	}

	// set values
	W_IDEX.ReadReg1Value = Regs[int(maskAndShift(src1Mask, instructionBits))]
	W_IDEX.ReadReg2Value = Regs[int(maskAndShift(src2Mask, instructionBits))]
	nonSignExtendedOffset := maskAndShiftShort(iConstMask, int16(instructionBits))
	W_IDEX.SEOffset = int32(nonSignExtendedOffset)

	// for rType instructions, WriteRegNum is bits 15-11
	W_IDEX.WriteRegNumR = maskAndShift(rDestMask, instructionBits)
	// for iType Instructions, writeRegNum is bits 20-16 (normally the src2 mask)
	W_IDEX.WriteRegNumAlt = maskAndShift(src2Mask, instructionBits)

}

func EX_Stage() {
	// register is coming in the with the function values from instruction:

	// NEED TO SWITCH ON ALUOp 10 for arithmetic, 00 for lb/sb
	switch R_IDEX.ALUOp {
		case : 0 { // Arithmetic: Either add, sub, or nop
			functionBits := maskAndShift(rFuncMask, EX.)
		}
	}

	// THE ALU receives the 5 function bits, and decodes them itself.

	// perform requested instruction indicated by info on READ side ID/EX

	// handle nop (do nothing)

	// handle lb/sb

	// handle add, sub

	// put values in WRITE side EX/MEM Register
}

func MEM_Stage() {
	// if instruction is a lb, use address calculated in EX, index into MM
	// get value there, put into MEM/WB
	if R_EXMEM.MemRead {
		// index into MM (think we use the ALU result for this)
		W_MEMWB.LBDataValue = Main_Memory[R_EXMEM.ALUResult]
	} else {
		// otherwise we're just passing information from EX/MEM to MEM/WB registers

		// TODO Should I dereference these?
		W_MEMWB.ALUResult = R_EXMEM.ALUResult
		W_MEMWB.MemToReg = R_EXMEM.MemToReg
		W_MEMWB.RegWrite = R_EXMEM.RegWrite
		W_MEMWB.WriteRegNum = R_EXMEM.WriteRegNum
	}

}

func WB_Stage() {
	// write to the registers from READ side MEM/WB

}

func print() {
	// TODO: need to format print of regs to correct 0x0 format, etc

	// read/write versions of each PipeReg
	/*
		fmt.Printf("Write IF/ID Reg: %+v\n", W_IFID)
		fmt.Printf("Read  IF/ID Reg: %+v\n", R_IFID)
	*/
	fmt.Println("CLOCK CYCLE: ", PC)
	fmt.Printf("Write ID/EX Reg: %+v\n\n", W_IDEX)
	fmt.Printf("Read  ID/EX Reg: %+v\n\n", R_IDEX)
	/*
		fmt.Printf("Write EX/MEM Reg: %+v\n", W_EXMEM)
		fmt.Printf("Read  EX/MEM Reg: %+v\n", R_EXMEM)

		fmt.Printf("Write MEM/WB Reg: %+v\n", W_MEMWB)
		fmt.Printf("Read  MEM/WB Reg: %+v\n", R_MEMWB)
	*/

}

func advanceRegisters() {
	// copy all write registers to their read counterparts
	// can i just do this or do i need to copy each value?
	// could just marshall the struct and unmarshall using json
	//CopyIFIDReg(&R_IFID, &W_IFID)
	R_IFID = W_IFID
	R_IDEX = W_IDEX
	R_EXMEM = W_EXMEM
	R_MEMWB = W_MEMWB
	//fmt.Println(R_IFID == W_IFID)
	//CopyReg(&R_IFID, &W_IFID)

}

/*
func CopyIFIDReg(read, write *IFIDReg) {
	*read = *write
}

func copyIDEXReg(read, write *IDEXReg) {
	*read = *write
}

func copyEXMEMReg(read, write *EXMEMReg) {
	*read = *write
}

func copyMemWBReg(read, write *MEMWBReg) {
	*read = *write
}
*/

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
	for i := uint32(1); i < 32; i++ {
		Regs[i] = 0x100 + i
	}
}

func initInstructionCache() {
	// from detailed example
	InstructionCache = []uint32{
		0x00a63820,
		0x8d0f0004,
		0xad09fffc,
		0x00625022,
		0x10c8fffb,
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,
	}
	/*
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
		}*/

	// start "program counter" at 0
	PC = 0
}

//// STRING METHODS
func (r Registers) String() string {
	registerStrings := make([]string, len(r))
	for i, _ := range r {
		registerStrings[i] = fmt.Sprintf("[$%.2d]: 0x%.3X\n", i, r[i])
	}
	return fmt.Sprintf(strings.Join(registerStrings, ""))
}

func (m Memory) String() string {
	memoryStrings := make([]string, len(m))
	for i, _ := range m {
		if m[i] == 0xFF {
			memoryStrings[i] = fmt.Sprintf("0x%X\n\n", m[i])
		} else {
			memoryStrings[i] = fmt.Sprintf("0x%X", m[i])
		}
	}
	return fmt.Sprintf(strings.Join(memoryStrings, " "))
}

func (r IDEXReg) String() string {
	formatString := "\n[RegDst: %v], [ALUSrc: %v], [ALUOp: 0x%X]\n[MemRead: %v], [MemWrite: %v], [MemToReg: %v]\n[Branch: %v],[RegWrite: %v]\nReadReg1Value: 0x%X\nReadReg2Value: 0x%X\nSEOffset: 0x%X\nWriteRegNumR: %d\nWriteRegNumAlt:%d"
	return fmt.Sprintf(formatString, r.RegDst, r.ALUSrc, r.ALUOp, r.MemRead, r.MemWrite, r.MemToReg,
		r.Branch, r.RegWrite, r.ReadReg1Value, r.ReadReg2Value,
		r.SEOffset, r.WriteRegNumR, r.WriteRegNumAlt)
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
