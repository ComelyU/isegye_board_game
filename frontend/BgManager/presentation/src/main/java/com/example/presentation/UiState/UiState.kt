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

data class OrderUiState(
    val orderId: Int,
    val orderQuantity: Int,
    val orderName: String
) {
    val orderIdString: String
        get() = orderId.toString()

    val orderQuantityString: String
        get() = orderQuantity.toString()
}