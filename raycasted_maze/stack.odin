package raycasted_maze

STACK_SIZE :: 1024

Stack :: struct($T: typeid) {
    data: [STACK_SIZE]T,
    head: i32
}

Push :: proc(stack: ^Stack($T), value: T) {
    stack.data[stack.head] = value
    stack.head += 1
}

Pop :: proc(stack: ^Stack($T)) -> T {
    stack.head -= 1
    return stack.data[stack.head]
}

IsEmpty :: proc(stack: ^Stack($T)) -> bool {
    return stack.head == 0
}