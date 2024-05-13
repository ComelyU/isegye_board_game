package com.example.isegyeboard.room_history

import android.content.Context
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.TextView
import androidx.recyclerview.widget.RecyclerView
import com.example.isegyeboard.R
import com.example.isegyeboard.room_history.model.OrderGameResponse

class GameHistoryAdapter(private val context: Context, private var dataList: List<OrderGameResponse>) :
    RecyclerView.Adapter<GameHistoryAdapter.HistoryViewHolder>() {

    // ViewHolder 클래스 정의
    inner class HistoryViewHolder(itemView: View) : RecyclerView.ViewHolder(itemView), View.OnClickListener {
        val nameTextView: TextView = itemView.findViewById(R.id.historyItemName)
        val quanTextView: TextView = itemView.findViewById(R.id.historyItemQuan)
        val orderStatusTextView: TextView = itemView.findViewById(R.id.orderStatus)

        override fun onClick(v: View?) {
            val history = dataList[adapterPosition]
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

        val orderStatusString = if (orderStatus == 0) {
            "주문 접수"
        } else if (orderStatus == 1) {
            "배송 중"
        } else {
            "배송 완료"
        }

        val orderTypeString = if (orderGameResponse.orderType == 0) {
            "배송 요청"
        } else {
            "반납 요청"
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
