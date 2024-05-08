package com.example.remote

import com.example.data.RemoteDataSource
import com.example.data.model.GameData
import com.example.data.model.OrderData
import com.example.data.model.OrderDetailData
import com.example.data.model.TurtleData
import com.example.remote.retrofit.ApiService
import javax.inject.Inject
import javax.inject.Singleton

@Singleton
internal class RemoteDataSourceImpl @Inject constructor(
    private val apiService: ApiService,
) : RemoteDataSource {

    override suspend fun getTurtleBot(): Result<TurtleData> = runCatching {
        val response = apiService.getTurtleBot()
        if (!response.isSuccessful) throw Exception()
        TurtleData(
            id = response.body()!!.id,
            storeId = response.body()!!.storeId
        )
    }

    override suspend fun getOrderList(): Result<List<OrderData>> = runCatching {
        val response = apiService.getOrderList("1")
        if (!response.isSuccessful) throw Exception()

        val ordersFromServer = response.body() ?: emptyList()
        val orders = ordersFromServer.map { serverOrder ->
            val orderDetailData = serverOrder.orderDetailData.map { serverDetail ->
                OrderDetailData(
                    orderDetailId = serverDetail.orderDetailId,
                    menuName = serverDetail.menuName,
                    quantity = serverDetail.quantity,
                    totalPrice = serverDetail.totalPrice
                )
            }
            OrderData(
                orderId = serverOrder.orderId,
                customerId = serverOrder.customerId,
                orderStatus = serverOrder.orderStatus,
                orderDetailData = orderDetailData
            )
        }
        orders
    }

    override suspend fun getGameList(): Result<List<GameData>> = runCatching {
        val response = apiService.getGameList("1")
        if (!response.isSuccessful) throw Exception()

        val games = response.body()!!.orderGameList.map { serverGame ->
            GameData(
                gameOrderId = serverGame.gameOrderId,
                customerId = serverGame.customerId,
                gameName = serverGame.gameName,
                stockLocation = serverGame.stockLocation,
                orderType = serverGame.orderType,
                orderStatus =serverGame.orderStatus
            )
        }
        games
    }
}