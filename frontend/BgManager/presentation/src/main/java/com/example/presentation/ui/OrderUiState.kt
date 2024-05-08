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
        get() = when (orderStatus) {
            0 -> "준비 중"
            1 -> "배송 중"
            2 -> "배송 완료"
            else -> "배송 오류"
        }
}