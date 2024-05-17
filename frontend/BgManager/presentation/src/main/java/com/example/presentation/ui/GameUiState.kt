package com.example.presentation.ui

data class GameUiState (
    val gameOrderId: Int,
    val customerId: Int,
    val roomNumber: Int,
    val gameName: String,
    val stockLocation: String,
    val orderType: Int,
    val orderStatus: Int,
) {
    val customerIdString: String
        get() = customerId.toString()

    val orderStatusString: String
        get() = when (orderStatus) {
            0 -> "준비 중"
            1 -> "배송 중"
            2 -> "배송 완료"
            3 -> "주문 취소"
            else -> "배송 오류"
        }

    val gameOrderIdString: String
        get() = "No.$gameOrderId"

    val orderTypeString: String
        get() = when (orderType) {
            0 -> "배송 주문"
            1 -> "반납 요청"
            else -> "주문 오류"
        }

    val roomNumberString: String
        get() = "$roomNumber 번 룸"
}