package com.example.data

import com.example.data.mapper.turtle.toDomain
import com.example.domain.model.OrderClass
import com.example.domain.model.TurtleBotClass
import com.example.domain.repository.Repository
import javax.inject.Inject
import javax.inject.Singleton

@Singleton
internal class RepositoryImpl @Inject constructor(
    private val remote: RemoteDataSource
) : Repository {
    override suspend fun turtleRepo(): Result<TurtleBotClass> {
        return remote.getTurtleBot().map { data ->
            data.toDomain()
        }
    }

    override suspend fun orderRepo(): Result<List<OrderClass>> {
        return remote.getOrderList().map { datalist ->
            datalist.map { it.toDomain() }
        }
    }
}