package com.example.presentation.ui

data class TurtleUiState(
    val turtleId: Int,
) {
    val turtleIdString: String
        get() = "$turtleId 호 선택"
}
