package com.example.data

import com.example.data.mapper.turtle.toData
import com.example.data.mapper.turtle.toDomain
import com.example.domain.model.DeliverClass
import com.example.domain.model.DeliverResponseClass
import com.example.domain.model.GameClass
import com.example.domain.model.OrderClass
import com.example.domain.model.BasicResponseClass
import com.example.domain.model.TurtleClass
import com.example.domain.repository.Repository
import javax.inject.Inject
import javax.inject.Singleton

@Singleton
internal class RepositoryImpl @Inject constructor(
    private val remote: RemoteDataSource,
//    private val local: LocalDataSource
) : Repository {
    override suspend fun turtleRepo(): Result<List<TurtleClass>> {
        return remote.getTurtleBot().map { datalist ->
            datalist.map { it.toDomain() }
        }
    }

    override suspend fun orderRepo(): Result<List<OrderClass>> {
        return remote.getOrderList().map { datalist ->
            datalist.map { it.toDomain() }
        }
    }

    override suspend fun gameRepo(): Result<List<GameClass>> {
        return remote.getGameList().map { datalist ->
            datalist.map { it.toDomain() }
        }
    }

    override suspend fun deliverRepo(deliverClass: DeliverClass): Result<DeliverResponseClass> {
        val deliverData = deliverClass.toData()
        return remote.startDeliver(deliverData).map { data ->
            data.toDomain()
        }
    }

    override suspend fun cancelGameRepo(gameOrderId: Int): Result<BasicResponseClass> {
        return remote.cancelGameOrder(gameOrderId).map { data ->
            data.toDomain()
        }
    }

    override suspend fun startMenuRepo(menuOrderId: Int): Result<BasicResponseClass> {
        return remote.startMenuOrder(menuOrderId).map { data ->
            data.toDomain()
        }
    }
    override suspend fun cancelMenuRepo(menuOrderId: Int): Result<BasicResponseClass> {
        return remote.cancelMenuOrder(menuOrderId).map { data ->
            data.toDomain()
        }
    }

//    override suspend fun getStoreInfo(storeId: Int): Result<BasicResponseClass> {
//        val localResponse =  local.getStoreInfo(storeId)
//        val response = if (localResponse.isSuccess) {
//            localResponse.map { data ->
//                data.toDomain()
//            }
//        } else {
//            remote.getStoreInfo(storeId).map { data ->
//                data.toDomain()
//            }
//        }
//        return response
//    }
}