package com.example.presentation

import android.os.Bundle
import androidx.appcompat.app.AppCompatActivity
import androidx.constraintlayout.widget.ConstraintLayout
import com.google.android.material.textfield.TextInputEditText

class LoginActivity : AppCompatActivity() {

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_login)

        val loginButton = findViewById<ConstraintLayout>(R.id.loginButton)
        val storeNumInput = findViewById<TextInputEditText>(R.id.storeNumField)
        val roomNumInput = findViewById<TextInputEditText>(R.id.roomNumField)

        loginButton.setOnClickListener {
            val storeId = storeNumInput.text.toString()
            val roomNum = roomNumInput.text.toString()

        }
    }
}