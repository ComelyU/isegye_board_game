package com.example.remote.di

import com.example.data.RemoteDataSource
import com.example.remote.RemoteDataSourceImpl
import dagger.Binds
import dagger.Module
import dagger.hilt.InstallIn
import dagger.hilt.components.SingletonComponent

@Module
@InstallIn(SingletonComponent::class)
internal abstract class RemoteDataSourceModule {
    @Binds
    abstract fun bindAuthenticationRemoteDataSource(
        sourceImpl: RemoteDataSourceImpl,
    ): RemoteDataSource
}