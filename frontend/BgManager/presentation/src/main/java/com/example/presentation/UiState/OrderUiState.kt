package com.example.presentation.UiState

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