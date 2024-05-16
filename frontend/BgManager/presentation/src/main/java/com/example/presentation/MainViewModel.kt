package com.example.presentation

import android.util.Log
import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.viewModelScope
import com.example.domain.model.DeliverClass
import com.example.domain.usecase.CancelUseCase
import com.example.domain.usecase.DeliverUseCase
import com.example.domain.usecase.GameUseCase
import com.example.domain.usecase.OrderUseCase
import com.example.domain.usecase.TurtleUseCase
import com.example.presentation.base.BaseViewModel
import com.example.presentation.ui.GameUiState
import com.example.presentation.ui.OrderDetailState
import com.example.presentation.ui.OrderUiState
import com.example.presentation.ui.TurtleUiState
import com.example.presentation.ui.UiState
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.launch
import javax.inject.Inject

@HiltViewModel
class MainViewModel @Inject constructor(
    private val turtleUseCase: TurtleUseCase,
    private val orderUseCase: OrderUseCase,
    private val gameUseCase: GameUseCase,
    private val deliverUseCase: DeliverUseCase,
    private val cancelUseCase : CancelUseCase
) :BaseViewModel<Unit>() {
    private val _uiStateLiveData = MutableLiveData<UiState>()
    val uiStateFlow: LiveData<UiState>
        get() = _uiStateLiveData

    init {
        _uiStateLiveData.value = UiState(emptyList(), emptyList(), emptyList())
    }

    private val _selectedTurtle = MutableLiveData<Int>()
    private val _selectedMenu = MutableLiveData<Int>()
    private val _selectedGame = MutableLiveData<Int>()
    private val _selectedReturn = MutableLiveData<Int>()
    private val _selectedRoom = MutableLiveData<Int>()
    val selectedTurtle: LiveData<Int> = _selectedTurtle
    val selectedMenu: LiveData<Int> = _selectedMenu
    val selectedGame: LiveData<Int> = _selectedGame
    val selectedReturn: LiveData<Int> = _selectedReturn
    val selectedRoom: LiveData<Int> = _selectedRoom

    var selectedTurtleString = "터틀봇 : ${selectedTurtle.value.toString()}"
    var selectedMenuString = "음료 : ${selectedMenu.value.toString()}"
    var selectedGameString = "게임 : ${selectedGame.value.toString()}"
    var selectedReturnString = "반납 : ${selectedReturn.value.toString()}"
    var selectedRoomString = "터틀봇 : ${selectedTurtle.value.toString()}"


    private val _showAlertDialogEvent = MutableLiveData<Event<String>>()
    val showAlertDialogEvent: LiveData<Event<String>>
        get() = _showAlertDialogEvent

    private fun showAlertDialog(message: String) {
        _showAlertDialogEvent.value = Event(message)
    }
    fun pushTurtle() {
        if (selectedTurtle.value == null) {
            Log.d("orderRequest", "주문 요청 내용: ${selectedTurtle.value}, ${selectedMenu.value}, ${selectedGame.value}, ${selectedReturn.value}")
            showAlertDialog("터틀봇을 선택해주세요.")
            return //터틀봇 선택 모달
        }

        Log.d("orderRequest", "주문 요청 내용: ${selectedTurtle.value}, ${selectedMenu.value}, ${selectedGame.value}, ${selectedReturn.value}")
        viewModelScope.launch {
            val deliverResponse = deliverUseCase.invoke(
                DeliverClass(
                    turtleId = selectedTurtle.value!!,
                    orderMenuId = selectedMenu.value,
                    orderGameId = selectedGame.value,
                    returnGameId = selectedReturn.value
                )
            )
            if (deliverResponse.isSuccess) {
                val deliverData = deliverResponse.getOrThrow()
                showAlertDialog("주문이 완료되었습니다.")
                Log.d("Deliver", "${deliverData.status}")
            } else {
                showAlertDialog("주문 요청을 실패했습니다.")
            }
        }
    }

    fun selectTurtle(turtleId: Int) {
        viewModelScope.launch {
            _selectedTurtle.value = turtleId
            Log.d("orderRequest", "터틀봇 갱신 : ${selectedTurtle.value}")
        }
    }

    fun selectMenuId(menuId: Int) {
//        if (_selectedRoom.value == )
        _selectedMenu.value = menuId
        Log.d("orderRequest", "메뉴 갱신 : ${selectedMenu.value}")
    }

    fun selectGame(gameId: Int, orderType: Int) {
        if (orderType == 0) {
            _selectedGame.value = gameId
            Log.d("orderRequest", "게임 갱신 : ${selectedGame.value}")
        } else {
            _selectedReturn.value = gameId
            Log.d("orderRequest", "반납 갱신 : ${selectedReturn.value}")
        }
    }

    fun cancelGameOrder(gameOrderId: Int) {
        viewModelScope.launch {
            val response = cancelUseCase.invoke(gameOrderId)
            if (response.isSuccess) {
                println(response.getOrThrow())
            }
        }
    }

    fun loadData() {
        viewModelScope.launch {
            // 터틀봇 목록 로드
            val turtleResponse = turtleUseCase.invoke()
            if (turtleResponse.isSuccess) {
                val turtleDataList = turtleResponse.getOrThrow()
                _uiStateLiveData.value = _uiStateLiveData.value?.copy(
                    turtles = turtleDataList.map { turtleData ->
                        TurtleUiState(
                            turtleId = turtleData.turtleId,
                        )
                    }
                )
            }

            //주문 목록 로드
            val orderResponse = orderUseCase.invoke()
            if (orderResponse.isSuccess) {
                val orderDataList = orderResponse.getOrThrow()
                _uiStateLiveData.value = _uiStateLiveData.value?.copy(
                    orders = orderDataList.map { orderData ->
                        OrderUiState(
                            orderId = orderData.orderId,
                            customerId = orderData.customerId,
                            orderStatus = orderData.orderStatus,
                            roomNumber = orderData.roomNumber,
                            orderDetail = orderData.orderDetail.map { it ->
                                OrderDetailState(
                                    orderDetailId = it.orderDetailId,
                                    menuName = it.menuName,
                                    quantity = it.quantity,
                                    totalPrice = it.totalPrice
                                )
                            }
                        )
                    }
                )
            }

            // 게임 목록 로드
            val gameResponse = gameUseCase.invoke()
            if (gameResponse.isSuccess) {
                val gameDataList = gameResponse.getOrThrow()
                _uiStateLiveData.value = _uiStateLiveData.value?.copy(
                    games = gameDataList.map { gameOrderData ->
                        GameUiState(
                            gameOrderId = gameOrderData.gameOrderId,
                            customerId = gameOrderData.customerId,
                            gameName = gameOrderData.gameName,
                            stockLocation = gameOrderData.stockLocation,
                            orderStatus = gameOrderData.orderStatus,
                            orderType = gameOrderData.orderType,
                            roomNumber = gameOrderData.roomNumber,
                        )
                    }
                )
            }
        }
    }
}

open class Event<out T>(private val content: T) {
    var hasBeenHandled = false
        private set // Allow external read but not write

    fun getContentIfNotHandled(): T? {
        return if (hasBeenHandled) {
            null
        } else {
            hasBeenHandled = true
            content
        }
    }
    fun peekContent(): T = content
}