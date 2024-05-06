package com.example.presentation.ui

data class OrderUiState(
    val orderId: Int,
    val customerId: Int,
    val orderStatus: Int,
    val orderDetail: List<OrderDetailState>
) {
    val customerIdString: String
        get() = customerId.toString()

    val orderStatusString: String
        get() = orderStatus.toString()
}