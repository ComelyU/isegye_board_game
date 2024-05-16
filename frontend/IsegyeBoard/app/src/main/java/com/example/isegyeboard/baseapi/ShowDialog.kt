package com.example.isegyeboard.baseapi

import android.content.Context
import androidx.appcompat.app.AlertDialog

object ShowDialog {
    fun showFailure(context: Context, message: String) {
        val builder = AlertDialog.Builder(context)

        builder.setTitle("요청 실패")
        builder.setMessage(message)

        builder.setPositiveButton("확인") {dialog, _ ->
            dialog.dismiss()
        }

        val dialog = builder.create()
        dialog.show()
    }
}