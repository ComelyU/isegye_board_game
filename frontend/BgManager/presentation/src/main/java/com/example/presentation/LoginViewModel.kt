package com.example.presentation

import androidx.lifecycle.viewModelScope
import com.example.domain.usecase.TurtleUseCase
import com.example.presentation.base.BaseViewModel
import kotlinx.coroutines.launch
import javax.inject.Inject

class LoginViewModel @Inject constructor(
    private val loginUseCase: TurtleUseCase
) : BaseViewModel<Unit>() {
    fun login() {
        viewModelScope.launch {
            val response = loginUseCase.invoke(

            )
            if (response.isSuccess) {
                val loginData = response.getOrThrow()

            }
        }
    }
}
