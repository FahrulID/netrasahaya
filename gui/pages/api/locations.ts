import { NextApiRequest, NextApiResponse } from "next";
import dbConnect from "@/lib/mongodb";
import Location from "@/models/location";

export default async function handler(
  req: NextApiRequest,
  res: NextApiResponse
) {
  await dbConnect();

  const { method } = req;
  const { id } = req.query;

  switch (method) {
    case "GET":
      try {
        if (id) {
          const location = await Location.findById(id);
          if (!location) {
            return res
              .status(404)
              .json({ success: false, error: "Location not found" });
          }
          res.status(200).json({ success: true, data: location });
        } else {
          const locations = await Location.find({});
          res.status(200).json({ success: true, data: locations });
        }
      } catch (error) {
        res.status(400).json({ success: false, error });
      }
      break;
    case "POST":
      try {
        const location = await Location.create(req.body);
        res.status(201).json({ success: true, data: location });
      } catch (error) {
        res.status(400).json({ success: false, error });
      }
      break;
    case "DELETE":
      try {
        const deletedLocation = await Location.findByIdAndDelete(id);
        if (!deletedLocation) {
          return res
            .status(404)
            .json({ success: false, error: "Location not found" });
        }
        res.status(200).json({ success: true, data: {} });
      } catch (error) {
        res.status(400).json({ success: false, error });
      }
      break;
    default:
      res.setHeader("Allow", ["GET", "POST", "DELETE"]);
      res.status(405).end(`Method ${method} Not Allowed`);
  }
}
