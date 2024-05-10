package com.example.remote

import com.example.data.RemoteDataSource
import com.example.data.model.DeliverData
import com.example.data.model.DeliverResponseData
import com.example.data.model.GameData
import com.example.data.model.OrderData
import com.example.data.model.OrderDetailData
import com.example.data.model.TurtleData
import com.example.remote.model.request.DeliverRequestModel
import com.example.remote.retrofit.ApiService
import javax.inject.Inject
import javax.inject.Singleton

@Singleton
internal class RemoteDataSourceImpl @Inject constructor(
    private val apiService: ApiService,
) : RemoteDataSource {

    override suspend fun getTurtleBot(): Result<List<TurtleData>> = runCatching {
        val response = apiService.getTurtleBot("1")
        if (!response.isSuccessful) throw Exception()

        val turtleFromServer = response.body() ?: emptyList()
        val turtles = turtleFromServer.map { serverTurtle ->
            TurtleData(
                turtleId = serverTurtle.id,
            )
        }
        turtles
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
                roomNumber = serverOrder.roomNumber,
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
                roomNumber = serverGame.roomNumber,
                stockLocation = serverGame.stockLocation,
                orderType = serverGame.orderType,
                orderStatus =serverGame.orderStatus
            )
        }
        games
    }

    override suspend fun startDeliver(
        deliverData: DeliverData
    ) : Result<DeliverResponseData> = runCatching {
        val response = apiService.startDelivery(
            turtleId = deliverData.turtleId,
            DeliverRequestModel(
                orderGameId = deliverData.orderGameId,
                orderMenuId = deliverData.orderMenuId,
                returnGameId = deliverData.returnGameId,
            )
        )
        if (!response.isSuccessful) throw Exception()

        DeliverResponseData(
            status = response.body()!!.status
        )
    }
}