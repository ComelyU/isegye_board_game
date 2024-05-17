package com.example.isegyeboard.room_history

import android.content.Context
import android.util.Log
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.Button
import android.widget.TextView
import androidx.recyclerview.widget.RecyclerView
import com.example.isegyeboard.R
import com.example.isegyeboard.baseapi.BaseApi
import com.example.isegyeboard.game_detail.GameOrderApi
import com.example.isegyeboard.room_history.model.OrderGameResponse
import retrofit2.Call
import retrofit2.Callback
import retrofit2.Response

class GameHistoryAdapter(private val context: Context, private var dataList: List<OrderGameResponse>) :
    RecyclerView.Adapter<GameHistoryAdapter.HistoryViewHolder>() {

    // ViewHolder 클래스 정의
    inner class HistoryViewHolder(itemView: View) : RecyclerView.ViewHolder(itemView), View.OnClickListener {
        val nameTextView: TextView = itemView.findViewById(R.id.historyItemName)
        val quanTextView: TextView = itemView.findViewById(R.id.historyItemQuan)
        val orderStatusTextView: TextView = itemView.findViewById(R.id.orderStatus)
        val cancelButton: TextView = itemView.findViewById(R.id.cancelButton)

        init {
            // cancelButton에 클릭 리스너 설정
            cancelButton.setOnClickListener(this)
        }

        override fun onClick(v: View?) {
            val orderId = dataList[adapterPosition].id.toString()

            val service = BaseApi.getInstance().create((GameOrderApi::class.java))
            Log.d("CancelOrder", "order id : $orderId")
            service.cancelOrder(orderId).enqueue(object : Callback<Void> {
                    override fun onResponse(call : Call<Void>, response: Response<Void>) {
                        if (response.isSuccessful) {
                            Log.d("CancelOrder", "Cancel Success : $response")
                            dataList = dataList.filterNot { it.id.toString() == orderId }
                            notifyDataSetChanged()
                            val editor = context.getSharedPreferences("RoomInfo", Context.MODE_PRIVATE).edit()
                            editor.remove("gameId")
                            editor.remove("isDeli")
                            editor.apply()
                        } else {
                            Log.d("CancelOrder", "Cancel Fail")
                        }
                    }
                    override fun onFailure(call: Call<Void>, t:Throwable) {
                        Log.d("CancelOrder", "$t")
                    }
            })
        }
    }

    // onCreateViewHolder: ViewHolder 객체를 생성하고 뷰를 연결
    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): HistoryViewHolder {
        val view = LayoutInflater.from(parent.context).inflate(R.layout.layout_history_order, parent, false)
        return HistoryViewHolder(view)
    }

    // onBindViewHolder: 데이터를 뷰에 연결
    override fun onBindViewHolder(holder: HistoryViewHolder, position: Int) {
        val orderGameResponse = dataList[position]

        val historyId = orderGameResponse.id
        val orderStatus = orderGameResponse.orderStatus
        val gameName = orderGameResponse.gameName

//        if (orderGameResponse.orderStatus == 3) {
//            holder.itemView.visibility = View.GONE
//            holder.itemView.layoutParams = RecyclerView.LayoutParams(0, 0)
//            return
//        } else {
//            holder.itemView.visibility = View.VISIBLE
//            holder.itemView.layoutParams = RecyclerView.LayoutParams(
//                ViewGroup.LayoutParams.MATCH_PARENT,
//                ViewGroup.LayoutParams.WRAP_CONTENT
//            )
//        }

        if (orderStatus == 0) {
            holder.cancelButton.visibility = View.VISIBLE
        } else {
            holder.cancelButton.visibility = View.GONE
        }

        val orderStatusString = when (orderStatus) {
            0 -> {
                "주문 접수"
            }
            1 -> {
                "배송 중"
            }
            2 -> {
                "배송 완료"
            }
            3 -> {
                "취소됨"
            }
            else -> {
                "배송 오류"
            }
        }

        val orderTypeString = if (orderGameResponse.orderType == 0) {
            "대여"
        } else {
            "반납"
        }

        holder.orderStatusTextView.append(orderStatusString)
        holder.nameTextView.text = "${gameName}\n"
        holder.quanTextView.text = orderTypeString
    }

    // getItemCount: 데이터 목록의 크기 반환
    override fun getItemCount() = dataList.size

    fun updateData(newHistoryList: List<OrderGameResponse>) {
        dataList = newHistoryList
        notifyDataSetChanged() // 변경 사항을 RecyclerView에 알림
    }
}
