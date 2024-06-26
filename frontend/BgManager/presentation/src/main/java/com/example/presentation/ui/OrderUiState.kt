package com.example.presentation.ui

data class OrderUiState(
    val orderId: Int,
    val customerId: Int,
    val orderStatus: Int,
    val roomNumber: Int,
    val orderDetail: List<OrderDetailState>
) {
    val orderIdString: String
        get() = "No.$orderId"

    val orderStatusString: String
        get() = when (orderStatus) {
            0 -> "준비 중"
            1 -> "메뉴 준비 중"
            2 -> "배송 중"
            3 -> "배송 완료"
            4 -> "주문 취소"
            else -> "배송 오류"
        }

    val roomNumberString: String
        get() = "$roomNumber 번 룸"
}