package com.example.presentation.UiState

data class UiState (
    val turtleId: Int,
    val storeId: Int,
    val orders: List<OrderUiState>
) {
    val turtleIdString: String
        get() = turtleId.toString()

    val storeIdString: String
        get() = storeId.toString()
}