.data 								#####  data for storing strings ######
	space:      .asciiz  " " 
	next_line:   .asciiz  "\n"
	is:			.asciiz  "is "
	k_msg:		.asciiz "Enter the value of k\n"
	n_msg:  	.asciiz "Enter the count of elements to be read\n"
	input_msg:  .asciiz "Enter the numbers seperated by newlines\n"
	print_msg:	.asciiz "th largest number among "
	
	arr: .space 80
	
.text
main:	
	addi $sp $sp -4  		#give 4 bytes to the stack to store the frame pointer
	sw   $fp 0($sp)  		#store the old frame pointer
	move $fp $sp     		#exchange the frame and stack pointers
	addi $sp $sp -92 		#allocate 92 more bytes of storage, 4 for $ra and 80 for our array 4 for k and 4 for n
	sw   $ra  -88($fp)
	
	li $v0 , 4				#display msg for k
	la $a0 ,k_msg			
	syscall
	
	li $v0 , 5				#reads k
	syscall
	sw $v0 , -84($fp)
	
	li $v0 , 4				#display msg for n
	la $a0 ,n_msg
	syscall
	
	li $v0 , 5				#reads n
	syscall
	sw $v0 , -80($fp)

	li $v0 , 4				#display msg for inputs
	la $a0 ,input_msg
	syscall
	
	lw $s0 , -80($fp)
	li $s1 , 0

read_input:
	beqz $s0 , sort
	
	li $v0 , 5
	syscall
	add $t0 , $s1 , $fp
	sw $v0 , ($t0)

	sub $s1 , $s1 , 4
	sub $s0 , $s0 , 1
	b read_input

sort:
	lw $s0 , -80($fp)				#store n in $s0
	li $s3 , -1						#initialize i to -1
	li $s4 , -1						#initialize j to -1
	b outer_loop

inner_loop:
	add $s4 , $s4 , 1				#j++
#	sub $t0 , $s0 , $s3				#t0 = n - i
	sub $t0 , $s0 , 1
	bge $s4 , $t0 , outer_loop		#if j >= n - i - 1 go to outerloop
	b cond
	

cond:
	mul $t0 , $s4 , 4 				#t0 = 4 * j
	add $t1 , $t0 , 4				#t1 = 4 * (j + 1)
	sub $t3 , $fp , $t0				#t3 = -4j(fp)
	sub $t4 , $fp , $t1 			#t4 = -4(j+1)(fp)
	lw  $t5 , ($t3) 				#t5 = arr[j]
	lw  $t6 , ($t4)					#t6 = arr[j+1]
	bgt $t5 , $t6 , swap 			#if t5 > t6 swap
	b inner_loop

swap:
	lw $t7 , ($t3) 					#t7 = arr[j]
	lw $t8 , ($t4)					#t8 = arr[j+1]
	sw $t7 , ($t4)					#arr[j+1] = t7
	sw $t8 , ($t3) 					#arr[j] = t8
	b inner_loop

set_j: 
	li $s4 , -1
	b inner_loop

outer_loop:
	add $s3 , $s3 , 1				# i++
	sub $t9 , $s0 , 1
	blt $s3 , $t9 , set_j			# i < n-1 go to inner loop

temp2:

	lw $a0 , -84($fp)
	li $v0 , 1
	syscall 

	li $v0 , 4				#display msg for printing
	la $a0 ,print_msg
	syscall

temp1: 	
	lw $s0 , -80($fp)
	li $s2 , 0

print:
	beqz $s0 , temp3
	
	add $t0 , $s2 , $fp

	lw $a0 , ($t0)
	li $v0 , 1
	syscall 

	li $v0 , 4				#display msg for seperater
	la $a0 ,space
	syscall

	sub $s2 , $s2 , 4
	sub $s0 , $s0 , 1
	b print

temp3:
	li $v0 , 4				#display msg for printing
	la $a0 ,is
	syscall

	lw $t0 , -84($fp)
	sub $t0 , $t0 , 1
	mul $t0 , $t0 , 4
	sub $t1 , $fp , $t0

	lw $a0 , ($t1)
	li $v0 , 1
	syscall 



exit: 								#### exit the program ####
	li $v0 , 4						#display msg for nextline
	la $a0 , next_line
	syscall
	li $v0 , 10 					#load exit command into $v0
	syscall 						#makes a system call

